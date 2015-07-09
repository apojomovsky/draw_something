### draw_something

Draw shapes using the trace left by turtlesim

### Synopsis

The draw_something library let you use the trace left by turtlesim for drawing figures.
These figures depend on the input files or recipes, where are specified the distances and angles in such an order that allows generating the desired figure.

### Code Example

In the "recipes" folder, are included some examples of ready-to-use recipes for testing purposes.

### Motivation

This project was made as a "school project", to learn the principles of using ROS and the publisher-subscriber communication paradigm.

### Dependencies

* [catkin](http://www.ros.org/wiki/catkin) - cmake and Python based buildsystem
* [ros_tutorials](https://github.com/ros/ros_tutorials) - needed for turtlesim

### Install

Get the code:

    git clone https://github.com/apojomovsky/draw_something.git

Build the workplace where you placed the repository:

    catkin_make

### Running the code

Excecute roscore:

    ~/roscore
    
In another terminal run a new instance of turtlesim:

    ~/rosrun turtlesim turtlesim_node
    
In another terminal run the draw_something program with the recipe file address as parameter:

    ~/rosrun draw_something draw_something <filename>
    
Have fun!

