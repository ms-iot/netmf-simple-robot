This project demonstrates the use of .NET Micro Framework to control a small robot.  The robot uses its onboard infrared sensors to detect the black outline of a circle against a white surface.

Hardware
===========
The robot uses the following hardware:
Netduino board – This is the board that runs the .NET Micro Framework.  Detailed information about the board is available at: http://netduino.com/
Pololu Robotics Zumo Arduino Robot Kit - The Zumo Robot kit provides the chassis, motor and sensors for robot locomotion and environment detection.  The robot kit and details on how to assemble can be found at: [http://www.pololu.com/](http://www.netmf.com)


Development Environment
=======================
Your development machine needs to be setup with Visual Studio
1)	Install Visual Studio (Express editions work fine).   You can download Visual Studio from here: http://www.visualstudio.com/en-us/products/visual-studio-express-vs.aspx
2)	Install the .NET Micro Framework SDK.  You can download the .NET Micro Framework from here:  http://netmf.codeplex.com/releases/view/133285
3)	Install the Netduino SDK.  Available here: http://netduino.com/
4)	Open the project in Visual Studio. 

Assembling the robot
=====================
Follow the instructions that come with the Pololu Zumo Arduino kit to assemble the robot.  Attach the Netduino board to the Zumo kit.

Setting up the surface for the robot to move in.
===============================================
The robot uses its onboard infrared sensors to detect the black outline of a circle against a white surface.  Black electrical tape is a good option for creating a circle or boundary around the robot.  The robot will stay within the boundary as long as there is sufficient contrast between the color of the surface the robot is on and the color of the electrical tape.  You can of course modify the thresholds to adjust for different surface types and colors of the tape you are using to create the boundary.

Additional Resources
=====================
For additional information on the .NET Micro Framework please visit  [http://www.netmf.com/](http://www.netmf.com)  
