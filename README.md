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

###### File structure:
```/config/config.yaml```: contain constants using for different nodes

```/launch/duck_imitator.launch```: launch file to run all the nodes to run the turtlebot

```/launch/test.launch```: launch file to run the nodes but only see the data changes printed out on the terminal and behavior of robot instead of having it move, only for testing purpose

```/src/avoid.py```: Avoid node so the baby duck can avoid obstacles, subscribed to /scan and /mv_cmd and publish to /cmd_vel

```/src/follow.py```: Follow node so the baby duck can follow its mama duck, subscribed to /heading_topic, publish to /mv_cmd

```/src/heading.py```: Heading node so the baby duck can determine the distance and orientation of the mama duck, subscribed to point_topic, publish to /heading_topic

```/src/yellow_detector.py```: Yellow_detector node so the baby duck can recognize yellow_square on the mama duck, subscribed to /camera/image, publish to /image_topic, /point_topic, reference link: https://github.com/computervisioneng/color-detection-opencv


```/src/utils.py```: Python file to contain scripts for utility functions using for different nodes

###### rqt_graph visualization:
![rqt_graph](/imgs/rosgraph.png)

###### Work distribution:
Diep (Emma) Vu: ```/config/config.yaml```, ```/src/heading.py```, ```/src/yellow_detector.py```, ```/src/utils.py```

Khai Dong: ```/launch/duck_imitator.launch```, ```/launch/test.launch```, ```/src/avoid.py```, ```/config/config.yaml```, ```/src/follow.py```

###### Note
Since Emma's account is busy running thesis code using python virtual environment, it cannot run roscore and bring up the robot. Hence Emma and Khai both use Khai's account to work on the project at the same time. You just have to trust us that we divided our work equally and ignore git commit history :smile:

###### Reference Links:
Videos: https://drive.google.com/drive/folders/1mKt4wA_nlJQJNu4LDfLjWiUSs0FW3EY-?usp=sharing

Gitlab Repo: https://cs-gitlab.union.edu/vud/csc325-final-project

Slides: https://docs.google.com/presentation/d/1IRhe8ga08-CutmjV4k5fmp90pnPTZu5ANoo-xW8VS7g/edit?usp=sharing






