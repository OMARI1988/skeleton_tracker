openni2_tracker
===============

`openni2_tracker` is a ROS Wrapper for the OpenNI2 and NiTE2 Skeleton Tracker.

### Installation
1. In your catkin_ws, clone skeleton tracker

    ```bash
    git clone https://github.com/OMARI1988/skeleton_tracker.git
    ```

2. Download NiTE-2.0.0 and OpenNI2 from the link below, and place them in your home directory.

    ```bash
    https://drive.google.com/open?id=0B3ZtY2aWnhsJQjdjdHk4RUtfeWc
    ```

3. Download Nite2 from the link below, and place it in your ~/.ros/ directory

    ```bash
    https://drive.google.com/open?id=0B3ZtY2aWnhsJUmVUYndfTXNTX2M
    ```

4. Get either the desktop version, or the robot version of the STRANDS system

    ```bash
    https://github.com/strands-project-releases/strands-releases/wiki
    ```

5. Get openni2 launch

    ```bash
    sudo apt-get install ros-indigo-openni2-launch
    ```

6. Finaly, catkin make your catkin_ws

    ```bash
    cd catkin_ws
    catkin_make
    ```

7. Now you should be able to run the skeleton tracker

    ```bash
    roslaunch skeleton_tracker tracker.launch camera:=head_xtion log_skeleton:=True message_store:=people_skeleton

    roslaunch openni2_launch openni2.launch camera:=head_xtion load_driver:=False depth_registration:=True debayer_processing:=True sw_registered_processing:=True
    ```


![marker](https://raw.githubusercontent.com/OMARI1988/skeleton_tracker/master/images/ex_1.jpg)

![marker](https://raw.githubusercontent.com/OMARI1988/skeleton_tracker/master/images/ex_2.jpg)

### THANKS!
Please let me know if something doesn't work, or if you have suggestions (or feel free to add stuff and send a pull request).
