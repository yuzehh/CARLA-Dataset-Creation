# CARLA-Dataset-Creation
As cooperative perception between vehicles and infrastructure (V2I) or vehicle and vehicle (V2V) gains prominence as an emerging research area, acquiring relevant multi-view data becomes crucial for researchers. However, obtaining such data can be challenging.
<!-- add carla link -->
This GitHub repository aims to tackle this challenge by providing code for collecting multi-view data in the CARLA simulator. The data collected using this code can be utilized for research on vehicle-infrastructure cooperative perception or connected vehicle cooperative perception, helping address the scarcity of multi-view data in the V2I and V2V cooperative perception domain.

Besides, we also public an infrastructure-vehicle multimodal dataset (~14GB) comprising mainstream sensor data (3D LiDAR point clouds and RGB images) and poses from both roadside infrastructure and vehicles. Access to the dataset for download through [this link (Google Drive)](https://drive.google.com/file/d/1xwg10Ueju2GhR2QSmSOABKSzGI0vIuxd/view?usp=sharing). 

## Introduction 
We support four different kinds of settings for collecting data in CARLA:
- Collect infrastrucutre-end data based on the existing lampposts in CARLA. There already exists multiple lampposts in CARLA, on which we can mount different sensors on and collect sensor data on the infrastructure's view. We visualize all lamppost postions of "Town05" in CARLA in the following image, we can find that not all the lampposts are on the roadside, there are some lampposts far away from the roadside. Moreover, the roadside lampposts are not evenly distributed, which is not in consistent with the real-world secenario settings.
- So we provide another option: Self define lampposts in CARLA and collect infrastrucutre-end data based on the self-defined lampposts. We define xxx lampposts in Town05 and visualize their postions in the following image. we can mount different sensors on these self-defined lampposts and collect sensor data on the infrastructure's view.
- Collect vehicle-end data. We support installing different sensors on an ego vehicle, collct sensor data on the vehicle's view with the existance of different levels of traffic conditions (heavy to light traffic) and with different driving speed.
- Collect multiple vehicle trajectories. We support spawning different numbers of vehicles and collect each of their trajectories.

We offer four different data collection settings in CARLA:

**Collecting Infrastructure-End Data from Existing Lampposts**: Utilizing the lampposts already present in CARLA, we can mount various sensors to gather sensor data from the infrastructure's view. However, it's worth noting that not all lampposts are situated roadside. Moreover, the roadside lampposts are not evenly distributed, which may deviate from real-world scenario settings. 

**Collecting Infrastructure-End Data Self-Defined Lampposts**: To address the limitations of existing lampposts, we provide an option to define custom lampposts in CARLA and mount sensors on these self-defined lampposts to collect infrastructure-end data. The positions of the xxx lampposts in Town05 are visualized in the provided image.

**Collecting Vehicle-End Data**: We support installing various sensors on an ego vehicle to collect vehicle-end data. This data collection can occur under different traffic conditions (ranging from heavy to light traffic) and at varying driving speeds.

**Collecting Multiple Vehicle Trajectories**: We provide support for spawning different numbers of vehicles and collecting the trajectories of each vehicle, which enables gathering valuable data on vehicle movements and interactions in various scenarios and traffic conditions.

These versatile data collection settings empower researchers to simulate diverse scenarios and conditions, enabling a more comprehensive exploration of cooperative perception between vehicles and infrastructure (V2I) or vehicle and vehicle (V2V) in CARLA.

## Getting Started
only use PythonAPI


## Citation
If you find the code or dataset useful, please cite our paper.
