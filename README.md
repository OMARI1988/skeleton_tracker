openni2_tracker
===============

`openni2_tracker` is a ROS Wrapper for the OpenNI2 and NiTE2 Skeleton Tracker. 

### Installation
1. In your catkin_ws, clone skeleton tracker

    ```bash
    git clone https://github.com/OMARI1988/skeleton_tracker.git
    ```
    
2. Download NiTE-2.0.0 and OpenNI2 from the link below, and place them in your home directoy.

    ```bash
    https://drive.google.com/open?id=0B3ZtY2aWnhsJQjdjdHk4RUtfeWc
    ```
    
3. Donwload Nite2 fron the link below, and place it in your ~/.ros/ directory

    ```bash
    https://drive.google.com/open?id=0B3ZtY2aWnhsJUmVUYndfTXNTX2M
    ```

4. Get either the desktop version, or the robot version of the STRANDS system

    ```bash
    https://github.com/strands-project-releases/strands-releases/wiki
    ```
    
5. Finaly, catkin make your catkin_ws

    ```bash
    cd catkin_ws
    catkin_make
    ```
    
6. Now you should be able to run the skelton tracker

    ```bash
    roslaunch skeleton_tracker tracker.launch
    ```


![marker](https://raw.githubusercontent.com/OMARI1988/skeleton_tracker/master/images/ex_1.jpg)

![marker](https://raw.githubusercontent.com/OMARI1988/skeleton_tracker/master/images/ex_2.jpg)

### THANKS!
Please let me know if something doesnt work, or if you have suggestions (or feel free to add stuff and send a pull request).
