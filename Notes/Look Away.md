# Look Away

## Description of Look Away

To see a ROS **subscriber** and **client** in action, you'll write a node called `look_away`. The `look_away`node will subscribe to the `/rgb_camera/image_raw` topic, which has image data from the camera mounted on the end of the robotic arm. Whenever the camera is pointed towards an uninteresting image - in this case, an image with uniform color - the callback function will request a `safe_move`service to safely move the arm to something more interesting. There are a few extra pieces in the code to ensure that this procedure is executed smoothly, but we’ll focus on those later.

## Updating the launch file

Just as you did with the `arm_mover` node, to get `look_away` to launch with the rest of the nodes, you will need to modify `robot_spawn.launch`, which can be found in`/home/workspace/catkin_ws/src/simple_arm/launch`. You can add the following code there:

```xml
  <!-- The look away node -->
  <node name="look_away" type="look_away" pkg="simple_arm"/>
```

Remember that a half turn of a joint requires pi/2 radians of revolution. Numerically, pi/2 is approximately 1.57. Since we want to be able to revolve a joint halfway around with one request, it will be helpful to set `max_joint_2_angle: 1.57` in `arm_mover`:

```xml
  <!-- The arm mover node -->
  <node name="arm_mover" type="arm_mover" pkg="simple_arm">
    <rosparam>
      min_joint_1_angle: 0
      max_joint_1_angle: 1.57
      min_joint_2_angle: 0
      max_joint_2_angle: 1.57
    </rosparam>
  </node>
```



