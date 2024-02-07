## Running instructions

>[!IMPORTANT]
> To run the simulation, Ros Noetic and Gazebo are required.
> Follow the [installation instructions](../README.md#installation) and [running instructions](../README.md#running-instructions) to install and run the simulation on a pre-configured Docker container (recommended).

### Start the simulation environment

Generate the environment
```
cd hospital_food_delivery/src/
python3 gen_env.py
```
Follow the prompts to configure the environmental conditions

Start the Gazebo simulation with the chosen configuration
```
roslaunch hospital_food_delivery mission_scenario.launch
```

### Run the simulation
> [!NOTE]  
> To run the experimentation three open terminals are required: the first one is the one already used to launch the Gazebo simulation; the second and third run the agents behavior.

#### Run agent 1
Open a new terminal window (**Terminal #2**)
    
Launch the robot controller for the agent 1
```
rosrun hospital_food_delivery robot_controller.py -ns 1
```

#### Run agent 2
Open a new terminal (**Terminal #3**)

Launch the robot controller for the agent 2
```
rosrun hospital_food_delivery robot_controller.py -ns 2
```

The simulation of the robot executing tasks will be shown in the Gazebo graphical environment.