# ROS wrapper for STag

## Building the code

To build the code, download it into the `src` directory of your catkin workspace. After downloading, you need to get the `STag` library by downloading the submodules. To do get the Stag submodule you need to initialize the git submodules.
```
cd stag_ros
git submodule update --init --recursive
```

## Usage

Note that for bundles, the order of corners is: top right, bottom left, bottom right, top left if you think of the bundle being on a flat sheet with the z-axis pointing towards you while you are looking at the sheet and X to your right and Y pointing towards the top of the sheet.

For example launch file see `launch/example.launch`

Currently this package only runs as a nodelet. It is able to process camera frames in parallel by changing the `num_worker_threads` parameter to your nodelet manager.

parameters:

* `marker_sizes_by_id`: a list of xmlrpc struct objects (json or yaml), each element with a value `size` which denotes the side length of a marker in meters and `id` whic is the id of the marker. Marker ids not in this list will be assumed to be the default marker size.
* `marker_bundles`: a list of marker bundles. Each marker bundle has a broadcasted id and a list of every member marker id and the corners of the marker in the bundles frame. If this parameter is missing, no marker bundles will be used. If it is present, all markers in a bundle will be grouped and used to solve an instance of the SolvePnP problem.
* `camera_info_topic`: The topic from which camera info will be read.
* `camera_image_topic`: The topic from which images will be read.
* `tag_id_type`: describes the class of id used by the STag library. See that code for more info. It determines the number of available ids.
* `default_marker_size`: Individual (those not in a bundle) markers which are not in the list `marker_sizes_by_id` are assigned this side length in meters.
* `output_frame_id`: `StagMarkers` messages are published with a `PoseStamped` for each marker. `output_frame_id` determines the frame the pose is defined in. There must be a transform between the camera's coordinate frame and this frame in the tf tree for this to work.
* `marker_message_topic`: the topic that marker messages are published to.
* `marker_frame_prefix`: When the marker coordinate frame is added into the `tf` tree, each marker will be given a name: "marker\_frame\_prefix" + \<marker\_id\>. For example, if `marker_frame_prefix` is `ar_marker_` and marker `1` is decodes, the marker will be `ar_marker_1` in the tf tree.
* `publish_debug_images`: if true, publishes images which show parts of the detection process overlayed on the image stream

For an example launch file and example parameter files, see the `launch` and `param` directories

# STag: A Stable Fiducial Marker System

Code used in the following paper:

[B. Benligiray; C. Topal; C. Akinlar, "STag: A Stable Fiducial Marker System," Image and Vision Computing (Accepted), 2019.](https://arxiv.org/abs/1707.06292)

Markers (will provide a generation script in the future):

https://drive.google.com/drive/folders/0ByNTNYCAhWbIV1RqdU9vRnd2Vnc

### TODO:
* Add a makefile
* Write Matlab and Python wrappers

[![Supplementary Video](https://user-images.githubusercontent.com/19530665/57184379-6a250580-6ec3-11e9-8ab3-7e139966f13b.png)](https://www.youtube.com/watch?v=vnHI3GzLVrY) 

Some figures from the paper:

<p align="center">
  <img src="https://user-images.githubusercontent.com/19530665/57179654-c0c11e00-6e88-11e9-9ca5-0c0153b28c91.png"/>
</p>

<p align="center">
  <img src="https://user-images.githubusercontent.com/19530665/57179660-cae31c80-6e88-11e9-8f80-bf8e24e59957.png"/>
</p>
