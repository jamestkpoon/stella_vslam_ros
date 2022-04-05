# Mirror Notes

1.
    ```
    docker build -f Dockerfile -t vslam:0.0.1 --build-arg NUM_THREADS=12 .
    ```
    - ./config folder is accessible at the env var ```CONFIG_PATH```
    - a [FBoW vocab file](https://github.com/stella-cv/FBoW_orb_vocab/blob/main/orb_vocab.fbow) is saved at the env var ```VOCAB_FILE_PATH```
1. 
    ```
    docker run -it -m 16g -v /tmp/.X11-unix/:/tmp/.X11-unix:ro -e DISPLAY=$DISPLAY vslam:0.0.1
    ```
    If there is some error similar to ```Error: Can't open display: :0```, try running ```xhost local:docker``` on the host machine
1. E.g. for ZED2:
    ```
    ros2 run stella_vslam_ros run_slam -v $VOCAB_FILE_PATH -c $CONFIG_PATH/zed2.yaml --ros-args --remap camera/left/image_raw:=/zed2/zed_node/left/image_rect_gray --remap camera/right/image_raw:=/zed2/zed_node/right/image_rect_gray
    ```
    - use of ```--remap``` is due to ```-r``` being parsed as a switch arg for stereo rectification


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
