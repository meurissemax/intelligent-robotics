# Introduction to intelligent robotics

The goal of this project is to program a robotic agent that gathers information about its environment using the sensors, plans a set of actions to respond appropriately to sensed data based on a pre-existing strategy, and executes a set of motor commands to carry out the actions that the plan calls for.

This project has been realized by :

* Maxime **Meurisse** - [m.meurisse@student.uliege.be](mailto:m.meurisse@student.uliege.be) - 20161278
* Valentin **Vermeylen** - [valentin.vermeylen@student.uliege.be](mailto:valentin.vermeylen@student.uliege.be) - 20162864

as part of the *Introduction to intelligent robotics* course given by Professor **Sacré** to the master students of Civil Engineering at the [University of Liège](https://www.uliege.be/) during the academic year 2019-2020, and uses the following resources to run :

* [MATLAB R2020a](https://www.mathworks.com/products/matlab.html) with :
	* [Robotics System](https://www.mathworks.com/products/robotics.html) toolbox
	* [Navigation](https://www.mathworks.com/products/navigation.html) toolbox
	* [Image Processing](https://www.mathworks.com/products/image.html) toolbox
* [TRS](http://ulgrobotics.github.io/trs/) : an open-source recipe for teaching/learning robotics with a simulator
* [VREP PRO EDU (version 3.6.2)](https://www.coppeliarobotics.com/previousVersions)

## Milestone navigation

In this milestone, the explore its environment and create an appropriate representation of it. This milestone has been realized in the file `navigation.m`.

## Milestone manipulation

In this milestone, the robot grasp some objects and bring them to a specific table. This milestone has been realized in the file `manipulation.m`.

## How to run the code

To run the code, the different resources used for this project (mentioned above) must be installed.

When all resources are installed :

1. Put the `trs/` folder (with appropriate modifications, available [here](http://ulgrobotics.github.io/trs/setup.html#install)) in the `matlab/` folder;
2. Launch MATLAB and run the `startup.m` script (by default, MATLAB should run it automatically);
3. Lauch VREP;
4. Update the `Values initialization` part of the `main.m` file (define the map dimensions, etc);
5. Run the `main.m` script.
