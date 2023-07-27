import glob
import os
import random
import sys
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import argparse
import copy
import logging
import random
from multiprocessing import Process
from queue import Empty, Queue

import carla
import cv2
import numpy as np
from carla import VehicleLightState as vls
from numpy import random


def spawn_points_near_intersections(save_path, threshold):
    global pole_num
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)  
    world = client.load_world('Town05')
    blueprint_library = world.get_blueprint_library()
    try:
        settings = world.get_settings()
        # We set CARLA syncronous mode
        settings.fixed_delta_seconds = 0.05 
        settings.synchronous_mode = True
        world.apply_settings(settings)

        spawn_points = world.get_map().get_spawn_points()
        crosswalks_list = world.get_map().get_crosswalks()
        crosswalks_points = np.zeros((1,3))
        for p in crosswalks_list:
            p = np.array((p.x, p.y, p.z)).reshape(-1,3)
            crosswalks_points = np.vstack(( crosswalks_points, p ))
        crosswalks_points = crosswalks_points[1:,:]
        # find the spawn points near the crosswalks
        sp_list = []
        for i,sp in enumerate(spawn_points):
            sp_location = np.array((sp.location.x, sp.location.y, sp.location.z))  
            distances = np.sqrt(np.sum(np.asarray(crosswalks_points-sp_location)**2, axis=1))
            if (distances <= threshold).any():
                sp_list.append(i)
        np.savetxt(save_path+"spindex_near_crosswalks_"+ str(threshold) + ".txt", np.array(sp_list))

    finally:
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        print("All cleaned up!")
        return np.array(sp_list)


def collect_lidar_data(sp_index, save_path, vehnum):
    sensor_list = []
    vehicle_position = []
    vehicles_id_list = []
    set_synchronous = True 
    IM_WIDTH = 1920
    IM_HEIGHT = 1920
    tm_port=8000
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)  
    # world = client.get_world()
    world = client.load_world('Town05')
    blueprint_library = world.get_blueprint_library()
    try:
        settings = world.get_settings()
        # We set CARLA syncronous mode
        settings.fixed_delta_seconds = 0.05 
        settings.synchronous_mode = True
        world.apply_settings(settings)

        #-------------------------- vehicle part --------------------------#
        tm = client.get_trafficmanager(tm_port)
        tm.set_global_distance_to_leading_vehicle(0) # 3
        # -- 30km/h * 1.5 = 45km/h 
        tm.global_percentage_speed_difference(-50.0) 
        tm.set_synchronous_mode(True)

        # spawn ego vehicle 
        spawn_points = world.get_map().get_spawn_points()
        blueprints = world.get_blueprint_library()
        ego_bp = blueprints.filter('charger_2020')[0]
        ego_vehicle = world.spawn_actor(ego_bp, spawn_points[sp_index])
        ego_vehicle.set_autopilot(True)

        # spawn other vehicles
        vehicle_bps = blueprints.filter('vehicle.*.*')
        vehicle_bps = [x for x in vehicle_bps if int(x.get_attribute('number_of_wheels')) == 4]

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot   
        SetVehicleLightState = carla.command.SetVehicleLightState
        FutureActor = carla.command.FutureActor

        batch = []
        number_of_vehicles = vehnum
        # near_idx = np.loadtxt('near_sp89_47_50m.txt')
        # chosen_idx = np.random.choice(near_idx, 10)
        # for i in chosen_idx:
        #     transform = spawn_points[int(i)]
        #     vehicle_bp = np.random.choice(vehicle_bps)
        #     vehicle_bp.set_attribute('role_name', 'autopilot')
        #     # spawn the cars and set their autopilot and light state all together
        #     batch.append(SpawnActor(vehicle_bp, transform).then(SetAutopilot(FutureActor, True, tm.get_port())))

        for n, transform in enumerate(spawn_points):
            if n >= number_of_vehicles:
                break
            if n == sp_index:
                continue
            vehicle_bp = np.random.choice(vehicle_bps)
            vehicle_bp.set_attribute('role_name', 'autopilot')
            # spawn the cars and set their autopilot and light state all together
            batch.append(SpawnActor(vehicle_bp, transform).then(SetAutopilot(FutureActor, True, tm.get_port())))

        for response in client.apply_batch_sync(batch, set_synchronous):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_id_list.append(response.actor_id)

        # get all vehicles 
        vehicles_list = world.get_actors().filter('vehicle.*')
        for v in vehicles_list:
            if v.attributes['role_name'] != "hero":
                # allow lane changing 
                tm.auto_lane_change(v, True)
                tm.ignore_lights_percentage(v, 100)

        #-------------------------- sensor part --------------------------#
        sensor_queue = Queue()
        cam_bp = blueprint_library.find('sensor.camera.rgb')
        spectator_cam_bp = blueprint_library.find('sensor.camera.rgb')
        slidar_bp = blueprint_library.find('sensor.lidar.ray_cast_semantic')
        lampslidar_bp = blueprint_library.find('sensor.lidar.ray_cast_semantic')
        # imu_bp = blueprint_library.find('sensor.other.imu')
        # gnss_bp = blueprint_library.find('sensor.other.gnss')
           #----------------------- vehicle sensors ---------------------#
        # set the attribute of camera
        cam_bp.set_attribute("image_size_x", "{}".format(IM_WIDTH))
        cam_bp.set_attribute("image_size_y", "{}".format(IM_HEIGHT))
        cam_bp.set_attribute("fov", "120")
        spectator_cam_bp.set_attribute("image_size_x", "{}".format(IM_WIDTH))
        spectator_cam_bp.set_attribute("image_size_y", "{}".format(IM_HEIGHT))
        spectator_cam_bp.set_attribute("fov", "120")
        # cam_bp.set_attribute('sensor_tick', '0.1')
    
        slidar_bp.set_attribute('upper_fov', '10.0')
        slidar_bp.set_attribute('lower_fov', '-30')
        slidar_bp.set_attribute('channels', '32') # 64 
        slidar_bp.set_attribute('points_per_second', '2600000') # 2600000
        slidar_bp.set_attribute('range', '30') 
        slidar_bp.set_attribute('rotation_frequency', str(int(1/settings.fixed_delta_seconds))) 

        cam = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=0, z=1.6)), attach_to=ego_vehicle)
        cam.listen(lambda data: sensor_callback(data, sensor_queue, "rgb"))
        sensor_list.append(cam)
        spectator_cam = world.spawn_actor(spectator_cam_bp, carla.Transform(carla.Location(x=0, y=0, z=30), carla.Rotation(yaw=0, pitch=-90, roll=0)), attach_to=ego_vehicle)
        spectator_cam.listen(lambda data: sensor_callback(data, sensor_queue, "rgb_spectator"))
        sensor_list.append(spectator_cam)
        slidar = world.spawn_actor(slidar_bp, carla.Transform(carla.Location(x=0, z=2.4)), attach_to=ego_vehicle)
        slidar.listen(lambda data: sensor_callback(data, sensor_queue, "slidar"))
        sensor_list.append(slidar)
           #----------------------- lamp sensors ---------------------#
        # lampslidar_bp.set_attribute('upper_fov', '0.0')
        # lampslidar_bp.set_attribute('lower_fov', '-60')
        # lampslidar_bp.set_attribute('channels', '64') # 64 * 5
        # lampslidar_bp.set_attribute('points_per_second', '2600000') # 4400000 * 5
        # lampslidar_bp.set_attribute('range', '50') 
        # lampslidar_bp.set_attribute('rotation_frequency', str(int(1/settings.fixed_delta_seconds))) 
        # for id, lampose in enumerate(lamp_poss):
        #     (lx,ly,lyaw) = lampose
        #     cam = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=lx, y=ly, z=5.2), carla.Rotation(yaw=lyaw, pitch=0, roll=0)))
        #     cam.listen(lambda data: sensor_callback( data, sensor_queue, "lamprgb_"+str(id) ))
        #     sensor_list.append(cam)
        #     slidar = world.spawn_actor(lampslidar_bp, carla.Transform(carla.Location(x=lx, y=ly, z=5), carla.Rotation(yaw=lyaw, pitch=0, roll=0)))
        #     slidar.listen(lambda data: sensor_callback( data, sensor_queue, "lampslidar_"+str(id) ))
        #     sensor_list.append(slidar)


        while True:
            if set_synchronous:
                world.tick()
                w_frame = world.get_snapshot().frame
                print("\nWorld's frame: %d" % w_frame)
                # vehicle location 
                v_location = ego_vehicle.get_transform().location
                v_rotation = ego_vehicle.get_transform().rotation # degrees pitch, yaw, roll 
                v_velocity = ego_vehicle.get_velocity()
                current_state = np.array([v_location.x, v_location.y, v_location.z, v_rotation.pitch, v_rotation.yaw, v_rotation.roll, v_velocity.x, v_velocity.y, v_velocity.z])
                print(current_state)
                filename = save_path + "vehicle_position/" + "sp_" + str(sp_index) + "/" + str(w_frame) + '.txt'
                mkdir_folder(save_path + "vehicle_position/", "sp_" + str(sp_index))
                np.savetxt(filename, current_state)
                if ego_vehicle.is_at_traffic_light():
                    traffic_light = ego_vehicle.get_traffic_light()
                    if traffic_light.get_state() == carla.TrafficLightState.Red:
                        print("Traffic light changed! Good to go!")
                        traffic_light.set_state(carla.TrafficLightState.Green)
                try:
                    for i in range (0, len(sensor_list)):
                        s_frame, s_name, s_data = sensor_queue.get(True, 1.0)
                        print("    Frame: %d   Sensor: %s" % (s_frame, s_name))
                        sensor_type = s_name
                        if sensor_type == 'rgb':
                            rgb = save_image(s_data)
                            filename = save_path + sensor_type + "/" + "sp_" + str(sp_index) + "/" + str(w_frame) + '.png'
                            mkdir_folder(save_path + sensor_type + "/", "sp_" + str(sp_index))
                            cv2.imwrite(filename, np.array(rgb[...,::-1]))
                        elif sensor_type == 'rgb_spectator':
                            rgb = save_image(s_data)
                            filename = save_path + sensor_type + "/" + "sp_" + str(sp_index) + "/" + str(w_frame) + '.png'
                            mkdir_folder(save_path + sensor_type + "/", "sp_" + str(sp_index))
                            cv2.imwrite(filename, np.array(rgb[...,::-1]))
                        elif sensor_type == 'slidar':
                            slidar = save_semanticlidar(s_data)
                            filename = save_path + sensor_type + "/" + "sp_" + str(sp_index) + "/" + str(w_frame)+'.npy'
                            mkdir_folder(save_path + sensor_type + "/", "sp_" + str(sp_index))
                            np.save(filename, slidar)
                        elif 'lamprgb' in sensor_type:
                            rgb = save_image(s_data)
                            filename = save_path + sensor_type + "/" + "sp_" + str(sp_index) + "/" + str(w_frame) + '.png'
                            mkdir_folder(save_path, sensor_type)
                            mkdir_folder(save_path + sensor_type + "/", "sp_" + str(sp_index))
                            cv2.imwrite(filename, np.array(rgb[...,::-1]))

                        elif 'lampslidar' in sensor_type:
                            slidar = save_semanticlidar(s_data)
                            filename = save_path + sensor_type + "/" + "sp_" + str(sp_index) + "/" + str(w_frame)+'.npy'
                            mkdir_folder(save_path, sensor_type)
                            mkdir_folder(save_path + sensor_type + "/", "sp_" + str(sp_index))
                            np.save(filename, slidar)

                except Empty:
                    print(" Some of the sensor information is missed ")

            else:
                world.wait_for_tick()

    finally:
        if set_synchronous:
            print('set mode to asynchronous')
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
            tm.set_synchronous_mode(False)
        for sensor in sensor_list:
            sensor.destroy()
        print('\ndestroying %d vehicles' % len(vehicles_id_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_id_list])
        print('done.')
        time.sleep(0.5)


def sensor_callback(sensor_data, sensor_queue, sensor_name):
    # Do stuff with the sensor_data data like save it to disk
    # Then you just need to add to the queue
    sensor_queue.put((sensor_data.frame, sensor_name, sensor_data))


def save_image(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    return array


def save_semanticlidar(point_cloud):
    """Prepares a point cloud with semantic segmentation
    colors ready to be consumed by Open3D"""
    data = np.frombuffer(point_cloud.raw_data, dtype=np.dtype([
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('CosAngle', np.float32), ('ObjIdx', np.uint32), ('ObjTag', np.uint32)]))

    points = np.array([data['x'], -data['y'], data['z']]).T
    labels = np.array(data['ObjTag']).reshape((-1,1))
    points = np.hstack((points,labels))

    return points 


def mkdir_folder(path, subfolder):
    if not os.path.isdir(os.path.join(path, subfolder)):
        os.makedirs(os.path.join(path, subfolder))
    return True


def main(save_folder='./', dis_thold=30, vehnum=500, collect_time=60):
    ''' dis_thold is the threshold of the distance (in meter) between spawn points and crosswalks. 
    If you want to spawn vehicles near crosswalks, set it to a small value; otherwise, set it to a large enough value to let the vehicle be spawned randomly. '''
    
    mkdir_folder(save_folder, 'vehicledata')
    # save_path = "/media/yuzeh/硬盘/CARLA_Dataset/23-2-5/vehicle_collect_smdens/"
    # load all annotated lamppost poses: n x 3 (xc,yc,yaw)
    # lamp_poss = np.loadtxt('lamppose_incarla.txt')
    # for spawn vehicles at random locations
    # for i in range(1):
    #     sp_index = random.randint(0, 254) 
    #     # sp_index = 32
    #     sp_index = 89
    #     # p = Process(target=collect_lidar_data, args=(sp_index, save_path, lamp_poss))
    #     p = Process(target=collect_lidar_data, args=(sp_index, save_path))
    #     p.start()
    #     # p.join()
    #     time.sleep(360)
    #     i += 1
    #     if p.is_alive():
    #         p.kill()
    #         time.sleep(10)

    # spawn vehicles around intersections within a distance of dis_thold
    sp_indexes = spawn_points_near_intersections(save_folder+'vehicledata/', dis_thold)
    sp = np.random.choice(sp_indexes)
    p = Process(target=collect_lidar_data, args=(int(sp), save_folder+'vehicledata/', vehnum))
    p.start()
    time.sleep(collect_time)
    if p.is_alive():
        p.kill()
        time.sleep(10)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')




