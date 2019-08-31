## Step 1 Create a Catkin Workspace

```
$ mkdir -p /home/workspace/catkin_ws/src
$ cd /home/workspace/catkin_ws/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
```

## Step 2 **Launch the robot inside your world**

```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch
```

## **Step 3- Run drive_bot and process_image**

This can be done by executing `ball_chaser.launch`:

```sh
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch
```

## Step 4- Visualize

To visualize the robot’s camera images, you can subscribe to camera RGB image topic from RViz. Or you can run the rqt_image_view node:

```sh
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ rosrun rqt_image_view rqt_image_view  
```