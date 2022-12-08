### Brief description
Use of ros amcl algorithm for global lozalization of an L1Br robot inside the LASER laboratory world.

### Installation 
Clone both the [*warehouse robot simulation*](https://github.com/LASER-Robotics/Warehouse_Gazebo) repository and this one to ros workspace src folder.

Install:
```
$ catkin_make
```

To run the algorithm, execute the following commands on different terminal [sessions](https://github.com/tmux/tmux/wiki):
```
$ roslaunch robot_description spawn.launch
```
```
$ roslaunch robot_control control.launch
```
```
$ roslaunch robotics_amcl amcl.launch
```