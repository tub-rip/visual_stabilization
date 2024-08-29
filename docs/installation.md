## Dependencies

### Install ROS Noetic (Ubuntu 20.04)

### Create a catkin workspace

Create a catkin workspace

	mkdir -p catkin_ws/src
	cd catkin_ws
	catkin config --init --mkdirs --extend /opt/ros/noetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release

The code has been tested on ROS Noetic. Depending on the ROS distribution you installed, you might have to use `kinetic` or `melodic` instead of `noetic` in the previous command.

### Add packages to the catkin workspace

**Clone** this repository into the `src` folder of your catkin workspace.

	cd catkin_ws/src
	git clone git@github.com:tub-rip/visual_stabilization.git

The catkin package dependencies are:
- [catkin simple](https://github.com/catkin/catkin_simple)
- ROS messages for DVS ([rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros))

The above dependencies are specified in the [dependencies.yaml](dependencies.yaml) file. They can be installed with the following commands from the `src` folder of your catkin workspace:

	cd catkin_ws/src
	sudo apt-get install python3-vcstool
	vcs-import < visual_stabilization/dependencies.yaml

The previous command should clone the repositories into folders *catkin_simple* and *rpg_dvs_ros* inside the src/ folder of your workspace. They should NOT be inside the *visual_stabilization* folder.

## Compiling

**Compile this package**:

	catkin build visual_stabilization
	
Source your workspace (at least the first time) by running:

	source ~/catkin_ws/devel/setup.bash

Sometimes (in case of strange errors) it is useful to delete the build folder before compilation:

	rm -rf build/visual_stabilization/

