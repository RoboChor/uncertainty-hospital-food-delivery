### To run the scenario

* In Terminal #1:
```
python3 /root/catkin_ws/src/hospital_food_delivery/src/gen_env.py; roslaunch hospital_food_delivery mission_scenario.launch
```

* In Terminal #2:
```
rosrun hospital_food_delivery robot_controller.py -ns 1
```

* In Terminal #3:
```
rosrun hospital_food_delivery robot_controller.py -ns 2
```