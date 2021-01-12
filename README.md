# Swerve2021

## *Description*

This example takes advantage of modern c++ to have modular code at each stage of the simulation.  Modular code makes it possible to swap out older code modules to newer ones with ease.  The module separation keeps each class simple and orthogonal which allows for easier collaboration.

## *Introduction*
This readme starts with a layout guide, and then gives a brief description of each folder section as such:

- Description
- Introduction (you are here)
  - Layout guide
  - Multiple projects
- Launcher
- Modules
  - Input
  - Robot
  - Output
- Properties

### Layout guide

The folder structure consists of the following groups:

- Base *- Framework base code that has proven to be useful for many years*
- Modules *- Complete sub projects with some form of unit testing*
  - Input *- Joystick controls, and AI goals*
  - Robot *- The heart of translating the input, converting processing, then sending to output*
  - Output *- Simulation viewer, WPI interface to robot*
- Properties *- A way to convert a deployable script for robot calibration*

## *Modules*

Each Module is self contained and can run independently to make this work if it's dependency is not linked yet it can default to some kind of unit test to act as the filler so that it can be maintained without need from any other module.

An overall view of the module layout consist of the input group, the robot group, and the output group.

### Input Group

- AI Input
- Input

The *Input* with its controller properties defines tele-op or the controller's implementation.  The Launcher will provide the Robot object and then input can bind its controls (specified by the properties)

*AI Input* is the autonomous that manages and executes goals.  If designed properly it should only need SLAM, and may need other sensors from the Robot object (which could be routed through SLAM)

The way the design currently stands is that launcher gives the time slices to the appropriate input.  The launcher manages which input module to use, so for example using the driver station app, the user picks autonomous or teleop, this gets passed to the launcher to pick if its going to pull the controllers, or run AI goals to push to the robot.  In either case, the selected input module can get a time slice for its operations.

### Robot Group

- Robot
- SLAM
- Swerve Robot super module

#### Robot

The *Robot* will receive input methods and push to output.  This is where the drive kinematics resides, it will use properties and if done correct should be able to survive multiple seasons.  The most common component of the robot is the rotary system, and so this is the front end for what is to be the output.

#### SLAM

*SLAM* (Simultaneous localization and mapping) is the object that keeps track of it's position on the field.  It will bind to robot to get sensor information (localization) to determine where it really is.  The mapping aspect can be addressed per game as needed and most-likely not a part of this module.

#### Swerve Robot super module

*Swerve Robot* the super module where it ties together various robot components into one module.  It contains all the odometry needed for localization.  *Note: SLAM is divided up, where localization occurs with odometry in the robot area, and the entity that records the position is managed outside the robot.  Mapping most-likely will be managed in the AI.*

### Output Group

- Simulated Output
- WPI Output

The simulated output can make use of SLAM data and present it in a way that can help see how the output will behave.  Typically in the past this has been used to monitor the drive from a top view 2D analyses of motion, but could indeed be translated to a 3D representation.  It can alternatively pull data from the Robot to show some representation of this data.  The SmartDashboard could be considered a simulated output.

Finally the WPI Output can pull data from the robot to update components as well as give data from sensors back to the robot, where the properties can help map out the slot assignments for each component.

## *Properties*

Properties represents a way to convert a deployable script for robot calibration, which is necessary for quick adjustments while in the pit.  This will most-likely be rewritten *(and probably should be)* by the students.  The example I included here will show conceptually how to interface with the modules where the robot does not have to do anything different or have any interaction with any given properties technique.  The way this works is the launcher will launch this first, and runs the entire script on startup.  To make life good, it may update on a reset or clicking start, in the past it updated when starting, but some parts required a reboot.  After consideration of keeping the robot code detached, and making it easy for properties to be passed over can be accomplished by use of an asset manager.  The methods of this are similar to that used in SmartDashboard, where they use simple primitive types, and the manager is very small lightweight.  It is used where the scripting method translates to write the asset database, and then the robot code can read from it.  This includes a registry file of all property names to help track down each entry on both the robot side and script loader.
