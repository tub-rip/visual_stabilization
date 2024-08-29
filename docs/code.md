
## Datasets
Our code inputs rosbags with events, images, and camera orientation (e.g., GT pose, IMU) data. The code has been tested with the following datasets:
- [RPG-ECD 2018](https://dsec.ifi.uzh.ch/).
- [MVSEC, RAL 2018](https://daniilidis-group.github.io/mvsec/)
- [VECtor, RAL 2022](https://star-datasets.github.io/vector/)
	- For our evaluation, we used the samples of the [Event-based Vision for VO/VIO/SLAM in Robotics](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM?tab=readme-ov-file#modified-vector-dataset) repository as it directly provides the data in rosbag format.

We suggest pre-processing the event data using the [DVS hot pixel filter](https://github.com/cedric-scheerlinck/dvs_tools/tree/master/dvs_hot_pixel_filter). It outputs bag files with a `.filtered` extension.

## Usage

Download a validation dataset in rosbag format (e.g., [`dynamic_6dof.bag`](https://download.ifi.uzh.ch/rpg/web/datasets/davis/dynamic_6dof.bag)). Please note that some sequences might not have all sensor data in a single rosbag file. We suggest this [script](https://gist.github.com/mfehr/305c1d07f6ca6a6e70afe1f155843d17) for rosbag merging.

### Frame stabilization

In a terminal run:

	roslaunch visual_stabilization frame_stabilization.launch

Play the rosbag file in another terminal. An rqt window should open and show the stabilized frames. The image below shows an example of the output: frames (left), stabilized frames (center), and stabilized frames in a big canvas (right) to provide a wider perspective of the stabilization effect.

<img src="media/frame_stabilization_output.gif" alt="Description" width="900px"/>


### Event stabilization

In a terminal launch:

	roslaunch visual_stabilization event_stabilization.launch

Play the rosbag file with event data. An rqt window will open to show the stabilized events on event images. Please note that event images are used only for visualization purposes. The video below shows an example of the expected output: undistorted events (left), stabilized events (center), and stabilized events over a large canvas (right).

<img src="media/event_stabilization_output.gif" alt="Description" width="900px"/>

## Parameters
The package inputs a set of configuration parameters to perform stabilization. We provide example configuration files for the three datasets mentioned above. The examples are located in the `visual_stabilization/config` folder. All code parameters are listed below:

### General
* **enable_undistortion**: Enable data (events or frames) undistortion
* **enable_saccades**: Enable saccadic effect 
* **canvas_gain**: Set canvas gain 
* **record_bag**: Enable rosbag output recording
* **file_name**: Path to ouput bag bafile

### Events
* **accumulation_type**: Event windowing approach (`time`, `number_of_events`, or `area`)
* **accumulation_time**:  Time window for event slicing (if `time` is selected)
* **number_of_accumulated_events**:  Number of events in the window (if `number_of_events` is selected)
* **roi_threshold**: Maximum number of events in a roi area (if `area` is selected)
* **number_of_columns**: Image rows 
* **number_of_rows**: Image columns

### Dataset 
* **image_width**
* **image_height**
* **K**: Camera matrix
* **D**: Distortion coefficients 
* **fisheye_distortion**: Activate in case of fish lens undistortion
* **topic_event_param**: Name of event topic
* **topic_image_param**: Name of image topic 
* **topic_external_pose_param**: Name of external camera pose topic 
* **get_orientation_from_imu**: Name of external imu topic   
* **use_external_extrinsic_rotation**: Activate in case an additional rotation is required to match the camera with the IMU
* **q_external**: Additional external rotation (quaternion)



