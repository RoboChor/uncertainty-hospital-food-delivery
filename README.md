## Instructions for installing the Adaptive mission control project
* Install dependencies for python
```
pip install docker 
```


* For obtaining the Docker image for Mission Control Adaptation
```
docker build . -t hfd:latest
```

* Start running the docker image
```
./gui-docker --name hfd_container hfd:v1.1 sleep infinity
```

*  (If you are running Linux) Start terminal #1
```
python3 open_terminal.py
```

This script will open a terminal#1, inside the terminal#1, run these commands and follow the instructions to set the environment values of the mission:
```
cd ~/catkin_ws/src/hospital_food_delivery/src/
python3 gen_env.py
```


For starting the gazebo simulation with the chosen specification, inside the terminal run the command:
```
roslaunch hospital_food_delivery mission_scenario.launch
```

* Start terminal #2 for running the agent 1 behaviour
```
python3 open_terminal.py
```
    
Inside the terminal #2, type the command:
```
rosrun hospital_food_delivery robot_controller.py -ns 1
```

* Start terminal #2 for running the agent 1 behaviour
```
python3 open_terminal.py
```
    
* Inside the terminal #2, type the command:
```
rosrun hospital_food_delivery robot_controller.py -ns 2
```

