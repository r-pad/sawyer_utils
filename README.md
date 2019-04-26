Sawyer Utilities
===

A collection of scripts and configurations used at RPAD.

# Running the code

## Running the simulation
* `source thing1.sh sim ` (May need to change IP to match your computer) 
* `roslaunch sawyer_utils sawyer_world.launch `
* In another terminal: `roslaunch sawyer_utils moveit_sim.launch`

# Installation
* Get prerequisite repositories into `catkin_ws/src` (See Rethink Wiki):
    * `intera_common`
    * `intera_sdk`
    * `sawyer_moveit`
    * `sawyer_robot`
    * `sawyer_simulator`
    * `sawyer_gazebo'
* Clone this repository into `src`
* Build and source the workspace

## Simulation installation
* Copy the models folder to `~/.gazebo/models`: `cp -r ~/catkin_ws/src/sawyer_utils/cfg/models/* ~/.gazebo/models/`
