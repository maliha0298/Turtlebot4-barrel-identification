# Turtlebot4-barrel-identification
The turtlebot will identify the barrel on its map (pgm file).

This script will use diameter of barrel, and inputs from yaml file to determine the circular barrel from pgm file. Using the resolution and origin from yaml file, this script will first determine which obstacle has area approximately close to area of barrel defined and then it will use origin coordinates to determine exact location of barrel.

Then using the BasicNavigator library the script will determine where the robot is and it will calculate how far it is from the barrel. And it will then move the robot rowards the barrel while keeping a safety distance of 0.4m.

Right now this script is for only one barrel.
