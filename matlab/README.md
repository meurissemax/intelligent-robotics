# Introduction to intelligent robotics

The goal of this project is to program a robotic agent that gathers information about its environment using the sensors, plans a set of actions to respond appropriately to sensed data based on a pre-existing strategy, and executes a set of motor commands to carry out the actions that the plan calls for.

This project has been realized by :

* Maxime **Meurisse** - [m.meurisse@student.uliege.be](mailto:m.meurisse@student.uliege.be) - 20161278
* Valentin **Vermeylen** - [valentin.vermeylen@student.uliege.be](mailto:valentin.vermeylen@student.uliege.be) - 20162864

and uses the following resources to run :

* [MATLAB R2020a](https://www.mathworks.com/products/matlab.html) with [Robotics system](https://www.mathworks.com/products/robotics.html) and [Navigation](https://www.mathworks.com/products/navigation.html) toolboxes
* [TRS](http://ulgrobotics.github.io/trs/) : an open-source recipe for teaching/learning robotics with a simulator
* [VREP PRO EDU (version 3.6.2)](https://www.coppeliarobotics.com/previousVersions)

## Milestone Navigation

In this milestone, we have programmed the robot to explore its environment and create an appropriate representation of it.

### How to run the code

To run the code, the different resources used for this project (mentioned above) must be installed.

When all resources are installed :

1. Put the `trs/` folder in this archive;
2. Launch MATLAB and run the `startup.m` script (by default, MATLAB should run it automatically);
3. Lauch VREP and open the scene;
4. Run the `navigation.m` script.
