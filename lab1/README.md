# EECE 5554 – Lab 1: Getting Familiar with ROS

In this lab, I successfully set up the required development environment for ROS 2 and GitHub and completed all tasks specified in the assignment.

First, I installed **Ubuntu 24.04 LTS (Noble Numbat)** and verified that my system environment, network connection, and basic Linux command-line tools are working correctly. I also installed **Git** and confirmed that I can use it from the terminal.

I created a **private GitHub repository** named `EECE5554`, set up the required `lab1` directory structure, and pushed all files using the command line. I added my Northeastern username text file stating that Ubuntu and Git are installed and that I am ready to use Git and GitHub for this course.

For the ROS portion of the lab, I:
- Installed **ROS 2 Jazzy** on Ubuntu.
- Configured the ROS environment.
- Used `turtlesim`, `ros2`, and `rqt` to explore ROS nodes, topics, and services.
- Drew a small doodle in turtlesim and saved the screenshot as `LI_turtle.png`.

I also completed the programming tasks by:
- Modifying the provided Python **publisher (talker)** node to publish a custom string message called "ROS2 is good for robotic studies!".
- Modifying the Python **subscriber (listener)** node so that it subscribes to the same topic and outputs
  `"I heard <modified message>"`, where the message is clearly altered from the original.

All required files are organized following the specified structure:

