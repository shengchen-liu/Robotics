# Project: Build My World

## Summary

1. Build a single floor wall structure using the **Building Editor** tool in Gazebo. Apply at least one feature, one color, and optionally one texture to your structure. Make sure there's enough space between the walls for a robot to navigate.
2. Model any object of your choice using the **Model Editor** tool in Gazebo. Your model links should be connected with joints.
3. Import your structure and two instances of your model inside an empty **Gazebo World**.
4. Import at least one model from the **Gazebo online library** and implement it in your existing Gazebo world.
5. Write a C++ **World Plugin** to interact with your world. Your code should display “Welcome to ’s World!” message as soon as you launch the Gazebo world file.

## How to run

1. Clone the repo

2. Create a build directory and compile the code

   ```
   $ cd ~/Udacity_Robotics/Robotics_Gazebo
   $ mkdir build
   $ cd build/
   $ cmake ../
   $ make # You might get errors if your system is not up to date!
   $ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/Udacity_Robotics/Robotics_Gazebo/build
   ```

3. Launch the world file in Gazebo to load both the world and the plugin

   ```
   $ cd ~/Udacity_Robotics/Robotics_Gazebo/world
   $ gazebo MyWorld
   ```

4. Visualize the output

   * A "Welcome to Shengchen's World!" message is printed in the terminal. This message interacts with the Gazebo World that includes your two-wheeled robot.

   * The world is visualized as following:

     ![](screen_shot.jpg)