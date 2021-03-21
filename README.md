# Baxter robot applications

Baxter is a cooperative robot (cobot) created by Rethink Robotics. It works both as an industrial robot an educational robots. Baxter its a humanoid robot, with a torso, two arms and a head (screen) giving him humanoid looks. As a cooperative robot, Baxter aims to help people at their workplace rather than replace them, and as such, its industrial capabilities are not optimized, in contrast, its safety measures are top grade, it does not require to be in a special room far from humans, nor being caged or any other safety measures that are usually taken with other industrial robots. This not only optimizes safety, but also space. You can check more about Baxter's safety measures here https://www.youtube.com/watch?v=t-ud55y4fLI

This git contains some basic functions and applications for the Baxter robot, as well as the basics you should know when you work with the robot.
The basics cover arm movement, head movement, changing the screen image, camera control and data collection, such as angles and positions.

Each file has comments explaining in a brief way how the apllication works. They also have the corresponding run instructions.

This git also contains my undergraduate's thesis project, a chess program that lets the Baxter robot play Chess against a human opponent.

DISCLAIMER: The chess code is not clean nor optimized. This happened due to the time restrictions given by the thesis. This will be fixed if I gain access to the robot in the future. Parts of the code, inclucing comments are in spanish, but they will be translated to english once the code's faults are fixed.

The chess project had the following objectives:
1) Integrating a chess engine with the baxter robot to make it able to play against human opponents.
2) Use a cooperative robot (cobot) to bridge the differences between human and robots, while at the same time, getting read of the fear and misconceptions people have about AI and robotics.
3) Show my knowledge and experience atained during my undergraduate years and obtain my diploma.

The chess engine was developed by Jean-Francois GAZET and you can find it here https://github.com/Tazeg/JePyChess. His personal website is the following: https://en.jeffprod.com/

The project was tested and implemented in Ubuntu 14.04, python 3 and ROS Indigo.
