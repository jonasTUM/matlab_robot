# MATLAB Robot

This project implements a manipulator with four degrees of freedom in *MATLAB*.

## Requirements
* The project was tested with *MATLAB* version *R2021a*.
* Required Toolboxes:
  * Aerospace Toolbox
  * Control System Toolbox
  * Navigation Toolbox
  * Optimization Toolbox
  * Robotics System Toolbox

## File Structure
* **main.m**: Script running the simulation.
* **environmentModel**: Files that define the environment and that perform collision checks.
* **pathPlanning**: Required files for the *RRT* planner.
* **pathPostProcessing**: Methods for improving the path quality of the *RRT* generated waypoints.
* **robot**: Kinematic robot model and robot collision model.
* **utils**: Utility files and global variables.
* **visualization**: Visualization files for the robot and the trajectory.
