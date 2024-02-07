# RoboChor Hospital Food Delivery multi-robot system exemplar

This repository contains the replication package of the paper **Handling uncertainty in the specification of autonomous multi-robot systems through mission adaptation** published at **19th International Conference on Adaptive and Self-Managing Systems (SEAMS 2024)**

Paper available at https://doi.org/10.1145/3643915.3644099

## Authors
This study has been designed, developed, and reported by the following investigators:
- Gianluca Filippone (University of L'Aquila, Italy)
- Juan Antonio Piñera García (Gran Sasso Science Institute, Italy)
- Marco Autili (University of L'Aquila, Italy)
- Patrizio Pelliccione (Gran Sasso Science Institute, Italy)

For any information, interested researchers can contact us by writing an email to [gianluca.filippone@graduate.univaq.it](mailto:gianluca.filippone@univaq.it)

## Mission Specification
### iHTN with adaptable task
![mission](</hospital-food-delivery/mission_img/hospital-food-delivery_mission.jpg>)

### Adaptation alternatives
#### Alternative #1
(To be activated if the inpatient is able to grab the food by himself/herself or if there is another human in the room)

![alternative 1](</hospital-food-delivery/mission_img/hospital-food-delivery_alternative1.jpg>)

#### Alternative #2
(To be activated if the inpatient is unable to grab the food and if there is no other human in the room)

![alternative 2](</hospital-food-delivery/mission_img/hospital-food-delivery_alternative2.jpg>)


## Repository Structure
```
uncertainty-hospital-food-delivery
|   README.md           # This file
|   Dockerfile          # Instructions for building the container for the simulation environment
|   gui-docker          # Run the container and launches the Gazebo environment
|   open_terminal.py    # Script aiding the launch of terminal inside the Docker containers   
└---hospital_food_delivery
    |   README.md       # Experiment running instructions
    |   CMakelists.txt  # Ros makefile
    |   package.xml     # Ros dependencies
    |---launch          # Ros implementation of the simulation
    |---src             # Ros implementation of the simulation
    |---worlds          # Environment models
    └---mission_img     # Mission specification HTN with adaptation alternatives


```

## Installation
### Requirements

Linux-based system (or VM), Pyton3, Git, Docker

### Download and build
Install docker (if it's not installed already) by following the steps from the [official website guide](https://docs.docker.com/engine/install/)

Follow the [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/) to give docker sudo privileges

Clone the repository
```
git clone https://github.com/RoboChor/uncertainty-hospital-food-delivery.git
```

Enter the directory
```
cd uncertainty-hospital-food-delivery
```

Build the Docker image
```
docker build . -t hfd
```

## Running instructions
### Start the simulation environment
Start the docker image
```
./gui-docker --name hfd_container hfd sleep infinity &
```

> [!WARNING]  
> Run `sudo chmod +x gui-docker` if unable to run the previous command and retry


Run the `open_terminal.py` script
```
python3 open_terminal.py
```

Run the command outputted by the script to access the terminal inside the docker container

> [!NOTE]  
> As alternative, the terminal inside the container can be accessed without running the `open_terminal` script. Instead, run `docker container ls` and substitute `[CONTAINER ID]` in the command below with the ID of the running hfd_container:
> ```
>docker exec -it [CONTAINER ID] bash
>```

Once accessed the terminal, generate the environment

```
cd ~/catkin_ws/src/hospital_food_delivery/src/
python3 gen_env.py
```
Follow the prompts to configure the environmental conditions

Start the Gazebo simulation with the chosen configuration
```
roslaunch hospital_food_delivery mission_scenario.launch
```

### Run the simulation
> [!NOTE]  
> To run the experimentation three open terminals on the Docker container are required: the first one is the one already used to launch the Gazebo simulation; the second and third run the agents behavior.
>
> The steps to access a new terminal inside the containers are the same as done before.

#### Run agent 1
Open a new terminal window (**Terminal #2**) and run the `open_terminal.py` script
```
python3 open_terminal.py
```
Run the command outputted by the script to access the terminal inside the docker container

> [!NOTE]  
> As explained before, the terminal inside the container can be accessed without running the `open_terminal` script through the command
> ```
>docker exec -it [CONTAINER ID] bash
>```
    
Launch the robot controller for the agent 1
```
rosrun hospital_food_delivery robot_controller.py -ns 1
```

#### Run agent 2
Open a new terminal (**Terminal #3**) for running the agent 2 behavior. Repeat the steps done before to access a new container terminal.
    
Launch the robot controller for the agent 2
```
rosrun hospital_food_delivery robot_controller.py -ns 2
```

The simulation of the robot executing tasks will be shown in the Gazebo graphical environment.