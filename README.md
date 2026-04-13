# Contactless Pulse Measurement with ROS
This repository is a student research project from three students from the Baden-Wuerttemberg Corporate State University Karlsruhe.

The approach of the project is to provide methods for contactless pulse measurement.

To reach this approach, the repository contains different modules:
* The common package: This package contains common classes that are used in multiple packages like the face detector.
* The eulerian_motion_magnification package: This package contains a contactless pulse measurement method which measures the pulse from little color changes in the face. 
* The legacy_measurement package: This package contains the implementation of a previous work on this topic. 
* The pulse_chest_strap package: This package enables pulse measurement with the PolarH7 chest strap. It can be used as ground truth for the contactless method.
* The pulse_head_movement package: This package contains a contactless pulse measurement method which measures the pulse from little head movements.

The usage of the individual packages is described in the following sections.

## Installation
The project is implemented in Python 3 and uses ROS 2 Humble Hawksbill. 

### Ubuntu
ROS recommends using Ubuntu 22.04, which you can find here https://releases.ubuntu.com/22.04/ .

### ROS 2
Install ROS 2 Humble following the official documentation: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html.

### Project workspace
Create a ROS 2 workspace and clone this repository:
```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recurse-submodules -b ros2-port https://github.com/l1sa3/PulsMeasurementStudien3.git
source /opt/ros/humble/setup.bash
```

Install colcon for building the ROS 2 workspace and camera info manager for camera calibration metadata:
```sh
sudo apt update
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-camera-info-manager
```

Now build the workspace with colcon: 
```sh
cd PulsMeasurementStudien3/
colcon build
source install/setup.bash
```

**Note**: In order to make colcon build succeed, you need to have pylon installed on the computer (see next step).  
Alternatively you can delete the src/pylon_ros2_camera directory, if you don't need the industry camera.

All python dependencies can be found in src/requirements.txt. Run the following in order to install them:
```sh
pip install -r src/pulse_head_movement/requirements.txt
```
Keep in mind: It can happen that a version defined in the requirements does not exist any more and is substituted by an api compatible newer version. In this case the versions numbers in the requirements have to be changed. 

### Industry Camera Driver (Basler pylon, ROS 2)
Download and install the pylon driver https://www.baslerweb.com/en/downloads/software/. (Version 7.2.1)
If you are using the tar.gz installer and want to install into `/opt/pylon`, follow the steps from the provided `INSTALL` file (simplified here):
```sh
cd ~/Downloads

# 1. Extract the pylon setup archive into a temporary directory
mkdir ./pylon_setup
tar -C ./pylon_setup -xzf ./pylon_*_setup.tar.gz

# 2. Go into the setup directory
cd ./pylon_setup

# 3. Extract the pylon SDK into /opt/pylon
sudo mkdir -p /opt/pylon
sudo tar -C /opt/pylon -xzf ./pylon_*.tar.gz
```

#### Basler blaze supplementary package (tar.gz)
The pylon ROS 2 driver in this repository also supports Basler blaze 3D cameras and expects the **pylon Supplementary Package for blaze** to be installed. Download the blaze package here: https://www.baslerweb.com/en/downloads/software/3178386853/ (Version 1.7.3) and install it as follows:

```sh
cd ~/Downloads

# 1. Extract the blaze setup archive into a temporary directory
mkdir ./blaze_setup
tar -C ./blaze_setup -xzf ./pylon-supplementary-package-for-blaze_*_setup.tar.gz

# 2. Go into the blaze setup directory
cd ./blaze_setup

# 3. Extract the blaze supplementary package into /opt/pylon
sudo tar -C /opt/pylon -xzf ./pylon-supplementary-package-for-blaze_*.tar.gz
```

After installation, the blaze headers should be available, e.g.:
```sh
ls /opt/pylon/include/pylon/BlazeInstantCamera.h
```

### Environment configuration for pylon
Configure the environment variables so that the compiler and runtime can find pylon and blaze (is recommended; we haven’t tested it yet):
```sh
echo 'export PYLON_ROOT=/opt/pylon' >> ~/.bashrc
echo 'export CMAKE_PREFIX_PATH=$PYLON_ROOT:$CMAKE_PREFIX_PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PYLON_ROOT/lib64' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```
You can test pylon with the pylon Viewer:
```sh
/opt/pylon/bin/pylonviewer
```

Install and initialize rosdep so that ROS2 dependencies can be resolved automatically if needed:
```sh
sudo apt update
sudo apt install python3-rosdep2
sudo rosdep init
rosdep update
```

Install the ROS2 dependencies required by the pylon ROS2 packages:
```sh 
cd ~/ros2_ws/src/PulsMeasurementStudien3/src/pylon_ros2_camera
rosdep install --from-paths . --ignore-src -r -y
```
You may experience some problems with the `diagnostic_updater` and `pcl_ros` dependencies. In this case, install them by executing the following commands:  
```
sudo apt install ros-humble-diagnostic-updater ros-humble-pcl-ros
```

Now compile the workspace using colcon 
```sh 
colcon build
```
**Note**: The --symlink-install flag can be added to the `colcon build` command. 
This allows the installed files to be changed by changing the files in the source space 
(e.g., Python files or other not compiled resourced) for faster iteration (refer to [the ROS2 documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)).

Source the environment:
```sh
source ~/ros2_ws/src/PulsMeasurementStudien3/install/setup.bash
```
Start the driver: 
```sh
ros2 launch pylon_ros2_camera_wrapper pylon_ros2_camera.launch.py
```
or
```sh
ros2 launch pylon_ros2_camera_wrapper my_blaze.launch.py to start the acquisition through the blaze.
```
For more information visit https://github.com/basler/pylon-ros-camera/tree/humble.

### PlotJuggler
If you want to display the output as graph, you need to install PlotJuggler.
Install the ROS 2 PlotJuggler packages via apt:
```sh
sudo apt update
sudo apt install ros-humble-plotjuggler ros-humble-plotjuggler-ros
```

Then start PlotJuggler for ROS 2, for example:
```sh
ros2 run plotjuggler plotjuggler
```

If that does not work on your system, see https://github.com/facontidavide/PlotJuggler for other installation possibilities.  
If you don't want  to install PlotJuggler you can also just print the results to the console.

## Measure Pulse from Head movement

You can measure the pulse from your head movement by using the pulse_head_movement package.<br/>
The method is inspired by http://people.csail.mit.edu/balakg/pulsefromheadmotion.html

You can run the head-movement pulse estimation node directly with a video file, for example:

```sh
source ros2_ws/src/PulsMeasurementStudien3/install/setup.bash
ros2 run pulse_head_movement pulse_head_movement \
  --ros-args \
  -p video_file:=/home/<user>/TestTest.mp4
```
Replace `/home/<user>/TestTest.mp4` with the path to your own test video file.

**Note**: The parts of this project below that use the Basler industry camera (pylon driver) or a webcam, as well as the comparison 
with the Polar H7 chest strap, could not be tested in this setup because no 
industry camera, no webcam and no Polar H7 device were used at this time.

### Run
There are three ways to execute the measurement:
1. Only display the measured values of the contactless method of head movement
2. Compare the values from the contactless method with a ground truth from the polarH7 chest strap. 
The pulse value from the polarH7 is measured in the pulse_chest_strap package.
3. Compare the values from the contactless method with a ground truth from the MAHNOB HCI Tagging Database.

#### Display pulse values only 
If you only want to get the pulse values from the contactless head movement method, you have two possibilities:
1. Print the pulse values to the console:
    ```sh
    source install/setup.bash
    # Run one of the following launch files
    # if you want to use the industry camera
    roslaunch pulse_head_movement industry_camera.launch
    # if you want to use the webcam
    roslaunch pulse_head_movement webcam.launch
    ```
2. Show the pulse values in PlotJuggler:
    ```sh
    source install/setup.bash
    # Run one of the following launch files
    # if you want to use the industry camera 
    roslaunch pulse_head_movement industry_camera.launch show_plot:=true
    # if you want to use the webcam
    roslaunch pulse_head_movement webcam.launch show_plot:=true
    ```
#### Compare pulse values with pulse values from polarH7
For comparison, the results are always displayed in Plotjuggler, so the installation of Plotjuggler is necessary here. This is currently only supported for the webcam.
```sh
source install/setup.bash
roslaunch common compare_pulse_values.launch topic:="/pulse_chest_strap" topic_to_compare:="/pulse_head_movement"
```
#### Compare pulse values with a video from the MAHNOB HCI Tagging Database
For comparison, the results are always displayed in Plotjuggler, so the installation of Plotjuggler is necessary here. This is currently only supported for the webcam.
```sh
source install/setup.bash
roslaunch common compare_pulse_values.launch topic:="/ecg" topic_to_compare:="/pulse_head_movement" video_file:="<path_to_video_file>" bdf_file="<path_to_bdf_file>"
```


## Measure Pulse with Eulerian Motion Magnification (Changing Colour Intensity)

The pulse can be measured by amplifying and filtering subtle colour changes in the face. This method is inspired by the Eulerian Motion Magnification from http://people.csail.mit.edu/mrub/papers/vidmag.pdf.

You can run the eulerian motion magnification pulse estimation node directly with a video file, for example:
```sh
source ros2_ws/src/PulsMeasurementStudien3/install/setup.bash
ros2 run eulerian_motion_magnification eulerian_motion_magnification \
  --ros-args \
  -p video_file:=/home/<user>/TestTest.mp4 \
  -p show_image_frame:=true
```
Replace `/home/<user>/TestTest.mp4` with the path to your own test video file.

### Run
This method can be run similarly to the head movement method:

#### Display pulse values only 
Only displaying the values from Eulerian Motion Magnification:
```sh
source install/setup.bash
# Run one of the following launch files
# if you want to use the industry camera
roslaunch eulerian_motion_magnification industry_camera.launch
# if you want to use the webcam
roslaunch eulerian_motion_magnification webcam.launch
```
If you want to display the values in PlotJuggler, launch PlotJuggler and subscribe to topic ```eulerian_motion_magnification```

#### Compare pulse values with pulse values from polarH7
```sh
source install/setup.bash
roslaunch common compare_pulse_values.launch topic:="/pulse_chest_strap" topic_to_compare:="/eulerian_motion_magnification"
```

#### Compare pulse values with a video from the MAHNOB HCI Tagging Database
```sh
source install/setup.bash
roslaunch common compare_pulse_values.launch topic:="/ecg" topic_to_compare:="/eulerian_motion_magnification" video_file:="<path_to_video_file>" bdf_file="<path_to_bdf_file>"
```

#### Show processed image
If you want to display the processed image, set the launch argument to ```show_processed_image:=true```, it is false by default.

## Measure Pulse with the legacy method

This method was implemented by another team of students (https://github.com/MobMonRob/PulsMeasurementStudien) and integrated in ROS within this repository.

### Run
This method can be run similarly to the head movement method:

#### Display pulse values only 
Only displaying the values from legacy method:
```sh
source install/setup.bash
# Run one of the following launch files
# if you want to use the industry camera
roslaunch legacy_measurement industry_camera.launch
# if you want to use the webcam
roslaunch legacy_measurement webcam.launch
```
If you want to display the values in PlotJuggler, launch PlotJuggler and subscribe to topic ```legacy_measurement```

#### Compare pulse values with pulse values from polarH7
```sh
source devel/setup.bash
roslaunch common compare_pulse_values.launch topic:="/pulse_chest_strap" topic_to_compare:="/legacy_measurement"
```

#### Compare pulse values with a video from the MAHNOB HCI Tagging Database
```sh
source install devel/setup.bash
roslaunch common compare_pulse_values.launch topic:="/ecg" topic_to_compare:="/legacy_measurement" video_file:="<path_to_video_file>" bdf_file="<path_to_bdf_file>"
```
