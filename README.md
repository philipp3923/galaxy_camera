# Galaxy Camera
ROS2 node for running cameras using the GalaxySDK. Currently not all camera configuration parameters are supported. New parameters can easily be added by modifiying the interface in `src/Camera.cpp`.

## Requirements
- ROS2 Iron
- OpenCV

## Installation
1. Download the [GalaxySDK](https://www.get-cameras.com/customerdownloads)
2. Install the SDK using the `Galaxy_camera.run`

## Configuration
The node can be configured using ROS2 parametrs. An example configuration can be found in `config/default.yaml`
