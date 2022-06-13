# Mirror Notes

1. cd to this repo
1. 
    ```
    docker build -f Dockerfile -t vslam:0.0.1 --build-arg NUM_THREADS=12 .
    ```
    - a [FBoW vocab file](https://github.com/stella-cv/FBoW_orb_vocab/blob/main/orb_vocab.fbow) is saved to ```VOCAB_FILE_PATH```
    - config/ is copied to ```CONFIG_PATH```, which is set to ```/vslam_config```
1. 
    ```
    docker run --rm -it -m 8g -u $(id -u):$(id -g) --net=host -v /dev/shm:/dev/shm -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID vslam:0.0.1
    ```
    - if you require X11, include ```-v /tmp/.X11-unix/:/tmp/.X11-unix:ro -e DISPLAY=$DISPLAY```
    - to bind mount this repository, include ```-v $(pwd):/ros2_ws/src/stella_vslam_ros```
        - to re-build with this bind mount: ```colcon build --packages-select stella_vslam_ros```
    - example of bind mounting a folder for saving maps:
        1. ```mkdir maps```
        1. ```sudo chmod 777 maps```
        1. include ```-v $(pwd)/maps:/maps```
        1. ```ros2 service call /vslam/save_map nav2_msgs/srv/SaveMap "{map_url: "/maps/test_map"}"```
1. ZED2 stereo:
    ```
    ros2 run stella_vslam_ros vslam_node --ros-args -p vocab:=$VOCAB_FILE_PATH -p config:=$CONFIG_PATH/zed2.yaml -p publish_tf:=false -p camera_frame:=zed2_left_camera_frame -r camera/left/image_raw:=/zed2/zed_node/left/image_rect_gray -r camera/right/image_raw:=/zed2/zed_node/right/image_rect_gray
    ```
    - if there is some error similar to ```Error: Can't open display: :0``` or ```Failed to open X display```:
        1. Exit the container
        1. ```xhost local:docker```
        1. Re-run the container

## vslam_node
### Added parameters
- ```config```: path to a config file, e.g. config/zed2.yaml
- ```vocab```: path to a vocab file
- ```rectify_stereo``` (default: false): rectify inbound stereo image pairs
- ```map```: loads a map file for localization if not empty

### Added services
- ```load_map```: load a map file
- ```save_map```: save a map file
- ```enable_mapping```: choose between SLAM (true) and localization only (false)



# stella_vslam_ros

[stella_vslam](https://github.com/stella-cv/stella_vslam)'s ROS package.

## Subscribed topics

### monocular setup

- `camera/image_raw`

### stereo setup

- `camera/left/image_raw`
- `camera/right/image_raw`

### RGBD setup

- `camera/color/image_raw`
- `camera/depth/image_raw`

## Published topics

- `~/camera_pose`

## Parameters

- `odom_frame`
- `map_frame`
- `base_link`
- `camera_frame`
- `publish_tf`
- `transform_tolerance`
- `use_exact_time` (stereo, RGBD only)

## Tutorial

Users unfamiliar with docker, ROS2, or stella_vslam should refer to the [tutorial](/doc/tutorial.md).
