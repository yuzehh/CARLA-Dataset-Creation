import glob
import os
import shutil
import sys
import time
from distutils.dir_util import copy_tree
from multiprocessing import Process

import cv2
import numpy as np

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
from queue import Empty, Queue

import carla
from carla import VehicleLightState as vls
from numpy import random


def mkdir_folder(path, subfolder):
    if not os.path.isdir(os.path.join(path, subfolder)):
        os.makedirs(os.path.join(path, subfolder))
    return True


def get_lamp_position(townname, save_folder):
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)  
    world = client.load_world(townname)
    all_lamp_info = np.zeros((1,6)) 
    try:
        env_poles = world.get_environment_objects(carla.CityObjectLabel.Poles)
        poleid_list = []
        for i, pole in enumerate(env_poles):  
            each_lidar_info = np.array([pole.transform.location.x, pole.transform.location.y, pole.transform.location.z, pole.transform.rotation.pitch, pole.transform.rotation.roll, pole.transform.rotation.yaw])  
            all_lamp_info = np.vstack((all_lamp_info, each_lidar_info))
            poleid_list.append(str(pole.id))
        all_lamp_info = all_lamp_info[1:,:]
        file_name = save_folder + 'lampincarla_poses.txt'
        np.savetxt(file_name, all_lamp_info)
        file_name = save_folder + 'lampincarla_IDlist.txt'
        with open(file_name, 'w') as f:
            f.write('\n'.join(poleid_list))
    finally:
        time.sleep(0.5)


def collect_lidar_data(townname, pole_id, save_path):
    sensor_list = []
    IM_WIDTH = 1920
    IM_HEIGHT = 1080
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)  
    world = client.load_world(townname)
    blueprint_library = world.get_blueprint_library()
    try:
        settings = world.get_settings()
        # We set CARLA syncronous mode
        settings.fixed_delta_seconds = 0.05 
        settings.synchronous_mode = True
        world.apply_settings(settings)

        env_poles = world.get_environment_objects(carla.CityObjectLabel.Poles)
        for p in env_poles:
            if str(p.id) == pole_id:
                pole = p
        assert pole is not None

        #-------------------------- sesor part --------------------------#
        sensor_queue = Queue()
        cam_bp = blueprint_library.find('sensor.camera.rgb')
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast_semantic')

        # set the attribute of camera
        cam_bp.set_attribute("image_size_x", "{}".format(IM_WIDTH))
        cam_bp.set_attribute("image_size_y", "{}".format(IM_HEIGHT))
        cam_bp.set_attribute("fov", "120")
        # cam_bp.set_attribute('sensor_tick', '0.1')
    
        # set the attribute of lidar
        lidar_bp.set_attribute('upper_fov', '0.0')
        lidar_bp.set_attribute('lower_fov', '-90.0')
        lidar_bp.set_attribute('channels', '128') # 64 * 5
        lidar_bp.set_attribute('points_per_second', '4400000') # 4400000 * 5
        lidar_bp.set_attribute('range', '80') 
        lidar_bp.set_attribute('rotation_frequency', str(int(1/settings.fixed_delta_seconds)))      
        # sensor_rotation = carla.Rotation(pitch=pole.transform.rotation.pitch-30, roll=pole.transform.rotation.roll, yaw=pole.transform.rotation.yaw)
        # for id, lampose in enumerate(lamp_poss):
        sensor_rotation = carla.Rotation(pitch=pole.transform.rotation.pitch, roll=pole.transform.rotation.roll, yaw=pole.transform.rotation.yaw)
        sensor_transform = carla.Transform(pole.transform.location + carla.Location(z=5), sensor_rotation)

        cam = world.spawn_actor(cam_bp, sensor_transform, attach_to=None)
        cam.listen(lambda data: sensor_callback(data, sensor_queue, 'lamprgb_'+pole_id))
        sensor_list.append(cam)
        slidar = world.spawn_actor(lidar_bp, sensor_transform, attach_to=None)
        slidar.listen(lambda data: sensor_callback(data, sensor_queue, 'lampslidar_'+pole_id))
        sensor_list.append(slidar)
        # world.debug.draw_box(carla.BoundingBox(pole0.transform.location,carla.Vector3D(0.5,0.5,2)), pole0.transform.rotation, 0.05, carla.Color(255,0,0,0),0)

        while True:
            # Tick the server
            world.tick()
            w_frame = world.get_snapshot().frame
            print("\nWorld's frame: %d" % w_frame)
            try:
                for i in range (0, len(sensor_list)):
                    s_frame, s_name, s_data = sensor_queue.get(True, 1.0)
                    print("    Frame: %d   Sensor: %s" % (s_frame, s_name))
                    sensor_type = s_name
                    if 'lamprgb' in sensor_type:
                            rgb =  _parse_image_cb(s_data)
                            filename = save_path + sensor_type + "/" + str(w_frame) + '.png'
                            mkdir_folder(save_path, sensor_type)
                            cv2.imwrite(filename, np.array(rgb[...,::-1]))
                    elif 'lampslidar' in sensor_type:
                        slidar = save_semanticlidar(s_data)
                        filename = save_path + sensor_type + "/" + str(w_frame)+'.npy'
                        mkdir_folder(save_path, sensor_type)
                        np.save(filename, slidar)

            except Empty:
                print("   Some of the sensor information is missed")

    finally:
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        # tm.set_synchronous_mode(False)
        for sensor in sensor_list:
            sensor.destroy()
        print("All cleaned up!")


def sensor_callback(sensor_data, sensor_queue, sensor_name):
    sensor_queue.put((sensor_data.frame, sensor_name, sensor_data))


def _parse_image_cb(image):
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


def list_txt(path, list=None):
    if list != None:
        file = open(path, 'w')
        file.write(str(list))
        file.close()
        return None
    else:
        file = open(path, 'r')
        rdlist = eval(file.read())
        file.close()
        return rdlist


def main(townname='Town05', save_folder='./lampincarla/', collect_time=10):
    mkdir_folder('./', 'lampincarla')  
    get_lamp_position(townname, save_folder)
    lamp_IDlist = save_folder + 'lampincarla_IDlist.txt'
    lamp_IDlist = [line.rstrip() for line in open(lamp_IDlist)]
    lampidxs = list(set(lamp_IDlist))
    for idx in lampidxs:
        mkdir_folder(save_folder, idx)
        p = Process(target=collect_lidar_data, args=(townname, idx, save_folder+idx+'/'))
        p.start()
        time.sleep(collect_time)
        if p.is_alive():
            p.kill()


if __name__ == '__main__':
    main()
