### CSC 325 Final Project: Duck Imitator
#### Authors: Diep (Emma) Vu + Khai Dong
##### Goal: Have turtlebot (baby duck) detect and follow another turtlebot with a yellow square on (mama duck) while avoid hitting obstacles.

##### List of Nodes (and corresponding Python files): yellow_detector, heading, avoid, follow 

##### Install dependencies:
```pip install -r requirements.txt```

##### Run the project and see the robot move:
```roslaunch duck-imitator duck-imitator.launch```

###### Run the test file to see the data changes but not actually having the robot to move (don't publish cmd_vel topic):
```roslaunch duck-imitator test.launch```
