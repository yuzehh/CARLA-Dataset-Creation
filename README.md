# CARLA-Dataset-Creation
As cooperative perception between vehicles and infrastructure (V2I) or vehicle and vehicle (V2V) gains prominence as an emerging research area, acquiring relevant multi-view data becomes crucial for researchers. However, obtaining such data can be challenging.
<!-- add carla link -->
This GitHub repository aims to tackle this challenge by providing code for collecting multi-view data in the [CARLA simulator](https://carla.org/). The data collected using this code can be utilized for research on vehicle-infrastructure cooperative perception or connected vehicle cooperative perception, helping address the scarcity of multi-view data in the V2I and V2V cooperative perception domain.

Besides, we also public an infrastructure-vehicle multimodal dataset (~14GB) comprising mainstream sensor data (3D LiDAR point clouds and RGB images) and poses from both roadside infrastructure and vehicles. Access to the dataset for download through [this link (Google Drive)](https://drive.google.com/file/d/1xwg10Ueju2GhR2QSmSOABKSzGI0vIuxd/view?usp=sharing). 

## Highlight Features

We offer four different data collection settings in CARLA:

**Collecting Infrastructure-End Data from Existing Lampposts**: Utilizing the lampposts already present in CARLA, we can mount various sensors to gather sensor data from the infrastructure's view. However, it's worth noting that not all lampposts are situated roadside. Moreover, the roadside lampposts are not evenly distributed, which may deviate from real-world scenario settings. 

**Collecting Infrastructure-End Data from Self-Defined Lampposts**: To address the limitations of existing lampposts, we provide an option to define custom lampposts in CARLA and mount sensors on these self-defined lampposts to collect infrastructure-end data. The positions of the xxx lampposts in Town05 are visualized in the provided image.

**Collecting Vehicle-End Data (near the intersection)**: We provide the option to spawn an ego vehicle near an intersection or not, as intersections present complex scenarios worthy of research. We support installing various sensors on the ego vehicle to collect vehicle-end data. This data collection process can occur under different traffic conditions (ranging from heavy to light traffic) and at varying driving speeds.

**Collecting Multiple Vehicle Trajectories**: We provide support for spawning different numbers of vehicles and collecting the trajectories of each vehicle, which enables gathering valuable data on vehicle movements and interactions in various scenarios and traffic conditions.

These versatile data collection settings empower researchers to simulate diverse scenarios and conditions, enabling a more comprehensive exploration of cooperative perception between vehicles and infrastructure (V2I) or vehicle and vehicle (V2V) in CARLA.

## Getting Started

### Download and Install CARLA
To use this repository, please first follow the official document to download CARLA: https://carla.readthedocs.io/en/0.9.13/. It's worth noting that you only need to download and install the packaged version of CARLA, and there is no need to build CARLA from the source. Building CARLA from source can be complicated and time-consuming, so opting for the packaged version simplifies the setup process. This repository is tested on CARLA version 0.9.13.

After the installation, download this repository and put all the files in this repository into the folder ```PythonAPI/examples```.

### Collecting Infrastructure-End Data from Existing Lampposts
First start the CARLA simulator by opening a terminal and running:
```
cd your_project_folder/CARLA_0.9.13/
./CarlaUE4.sh  # start CARLA simulator
```

Then open another terminal and run:
```
cd your_project_folder/CARLA_0.9.13/PythonAPI/
python3 carla_infra_datacollect.py
```

This will create a folder named "lampincarla", which comprises three components:

1. A file named 'lampincarla_poses.txt', containing the positions (x, y, z) and poses (pitch, roll, yaw) of all the lampposts.
2. A file named 'lampincarla_IDlist.txt', including the IDs of all the lampposts.
3. Multiple sub-folders, each named by "sensortype_lampID", which contain the sensor data of the lampposts with specific IDs.

Within the script, you have the flexibility to modify the scenario (Town name), data collection time, and add or remove sensor types mounted on the infrastructure. Additionally, you can adjust the parameters and configurations of the sensors.


### Collecting Infrastructure-End Data from Self-Defined Lampposts

We have pre-defined 181 lampposts in the "Town05" in CARLA. The file 'lamp_selfdefined.txt' contains a 181 x 3 array, representing the information (x, y, yaw) of all these self-defined lampposts. Each lamppost is assigned a unique ID, which corresponds to its index in this array.

First start the CARLA simulator by opening a terminal and running:
```
cd your_project_folder/CARLA_0.9.13/
./CarlaUE4.sh  # start CARLA simulator
```

Then open another terminal and run:
```
cd your_project_folder/CARLA_0.9.13/PythonAPI/examples
python3 selfdefined_infra_datacollect.py
```

This will generate a folder named "selfdefined_lamp". This folder will contain multiple sub-folders, each named by "sensortype_lampID," housing the sensor data of the lampposts with specific IDs. The script offers you the flexibility to modify the data collection time, add or remove sensor types mounted on the infrastructure, and adjust the parameters and configurations of the sensors as needed. 


### Collecting Vehicle-End Data

First start the CARLA simulator by opening a terminal and running:
```
cd your_project_folder/CARLA_0.9.13/
./CarlaUE4.sh  # start CARLA simulator
```

Then open another terminal and run:
```
cd your_project_folder/CARLA_0.9.13/PythonAPI/
python3 vehicle_datacollect.py
```

Upon execution, this script will generate a folder named "vehicledata" that consists of several components. First, it extracts spawn points near intersections within a specified distance *threshold* and saves them to a list named 'spindex_near_crosswalks_threshold.txt'. This list contains the indexes of spawn points located near intersections within the defined distance *threshold*.

Next, the script randomly selects a spawn point from the list to spawn the ego vehicle. The ego vehicle then starts collecting sensor data while in motion. The collected sensor data is organized into four sub-folders within "vehicledata":

1. "vehicle_position": Contains information about the ego vehicle (x, y, z, pitch, yaw, roll, velocity_x, velocity_y, velocity_z) for each frame.
2. "rgb": Contains RGB images captured by the vehicle's camera for each frame.
3. "slidar": Contains the point cloud data from the vehicle's LiDAR sensor for each frame.
4. "rgb_spectator": Contains RGB images captured by a spectator camera, which moves in sync with the ego vehicle to provide bird-eye-view images for each frame.

Additionally, the script offers flexibility to customize various parameters and configurations, including the intersection distance *threshold*, the number of surrounding vehicles, data collection time, sensor types on the vehicle, and other sensor-related settings.

### Collecting Multiple Vehicle Trajectories

First start the CARLA simulator by opening a terminal and running:
```
cd your_project_folder/CARLA_0.9.13/
./CarlaUE4.sh  # start CARLA simulator
```

Then open another terminal and run:
```
cd your_project_folder/CARLA_0.9.13/PythonAPI/
python3 vehicles_wanderfortraj.py
```

This script will generate a folder named "vehicle_trajs", which contains the trajectory data of each spawned vehicle. Each trajectory file is represented as an nx10 array, where n denotes the frame numbers, and each row contains the information (x, y, z, pitch, yaw, roll, frame_index, velocity_x, velocity_y, velocity_z). The script provides flexibility in both the number of spawned vehicles and the data collection time. 

## Citation

This repository is a by-product of [VI-Map](https://github.com/yuzehh/VI-Map). If you find the code or dataset useful, please cite the paper VI-Map.

