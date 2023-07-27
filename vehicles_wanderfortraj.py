
import glob
import os
import sys
from random import sample

try:
    sys.path.append(glob.glob('../../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import logging
import math
import multiprocessing
import queue
import random
import time
from multiprocessing import Process

import carla
import cv2
import numpy as np


def mkdir_folder(path, subfolder):
    if not os.path.isdir(os.path.join(path, subfolder)):
        os.makedirs(os.path.join(path, subfolder))
    return True


def generate_trajs(save_folder, vehnum, collect_time):
    vehicles_id_list = []
    set_synchronous = True 
    tm_port=8000
    timing=0

    try:
        traj_record = {}
        client = carla.Client('localhost', 2000)
        client.set_timeout(5.0)
        # world = client.get_world()
        world = client.load_world('Town05')
        tm = client.get_trafficmanager(tm_port)
        tm.set_global_distance_to_leading_vehicle(2.5)
        tm.global_percentage_speed_difference(-80.0) 

        if set_synchronous:
            print('Pay attention this client set mode to synchronous')
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)
            tm.set_synchronous_mode(True)

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        SetVehicleLightState = carla.command.SetVehicleLightState
        FutureActor = carla.command.FutureActor
        blueprint_library = world.get_blueprint_library()
        blueprints_vehicle = blueprint_library.filter("vehicle.*")
        blueprints = sorted(blueprints_vehicle, key=lambda bp: bp.id)
        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)
        # --------------
        # Spawn vehicles
        # --------------
        
        batch = []
        number_of_vehicles = vehnum
        for n, transform in enumerate(spawn_points):
            if n >= number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')

            # spawn the cars and set their autopilot and light state all together
            batch.append(SpawnActor(blueprint, transform)
                .then(SetAutopilot(FutureActor, True, tm.get_port())))

        for response in client.apply_batch_sync(batch, set_synchronous):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_id_list.append(response.actor_id)

        # get all vehicles 
        vehicles_list = world.get_actors().filter('vehicle.*')
        for v in vehicles_list:
            if v.attributes['role_name'] != "hero":
                tm.auto_lane_change(v, True)

        sample_index = sample(range(vehnum), 50)
        for i in sample_index:
            v = vehicles_list[i]
            if v.attributes['role_name'] != "hero":
                tm.force_lane_change(v, True)

        for id in vehicles_id_list:
            traj_record[id] = list()
        while True:
            timing = timing + 1 
            if timing > collect_time * 20:
                break
            if set_synchronous:
                world.tick()
                w_frame = world.get_snapshot().frame
                # set all traffic lights to green
                vehicles_list = world.get_actors().filter('vehicle.*')
                for v in vehicles_list:
                    v_location = v.get_transform().location
                    v_rotation = v.get_transform().rotation # degrees pitch, yaw, roll 
                    v_velocity = v.get_velocity()
                    current_state = np.array([v_location.x, v_location.y, v_location.z, v_rotation.pitch, v_rotation.yaw, v_rotation.roll, w_frame, v_velocity.x, v_velocity.y, v_velocity.z])
                    v_traj = traj_record[v.id]
                    v_traj.append(current_state)
                    traj_record[v.id] = v_traj
                # traj_record = eval(traj_record)
                # with open ('vehicle_trajectories.txt', 'w') as f:
                #     f.write(str(traj_record))
                # np.save('vehicle_trajectories.npy', traj_record)
                #     if v.is_at_traffic_light():
                #         traffic_light = v.get_traffic_light()
                #         if traffic_light.get_state() == carla.TrafficLightState.Red:
                #             # world.hud.notification("Traffic light changed! Good to go!")
                #             traffic_light.set_state(carla.TrafficLightState.Green)

            else:
                world.wait_for_tick()

    finally:
        vals = traj_record.values()
        traj_num = len(vals)
        print(" ------------- Trajectory Number: ", traj_num, "-----------------")
        for i,val in enumerate(vals):
            trajs = np.zeros((1,10))
            each_traj = np.array(val).reshape((-1,10))
            trajs = np.vstack((trajs, each_traj))
            trajs = trajs[1:, :]
            np.savetxt(save_folder+'vehicle_trajs/'+'trajectory_' + str(i) + '.txt', trajs)

        if set_synchronous:
            print('set mode to asynchronous')
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
            tm.set_synchronous_mode(False)
        print('\ndestroying %d vehicles' % len(vehicles_id_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_id_list])
        print('done.')
        time.sleep(0.5)
        

def main(save_folder='./', vehnum=200, collect_time=30):
    mkdir_folder(save_folder, 'vehicle_trajs')
    generate_trajs(save_folder, vehnum, collect_time)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')


