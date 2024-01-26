import os
import json
import argparse


def main(args=None):

    # Open and read the JSON file
    if args is None:
        with open('/root/catkin_ws/src/hospital_food_delivery/worlds/status.json', 'r') as file:
            data = json.load(file)
            sca = data["self_care_ability"]
            alo = data["alone"]
    else:
        sca = args["self_care_ability"]
        alo = args["alone"]
    file_location = "/root/catkin_ws/src/hospital_food_delivery/launch/mission_scenario.launch"
    
    sel = str(sca) + str(alo)
    ps_sels = ["00","01","10","11"]
    
    with open(file_location, 'r') as file:
        s = file.read()
    
    # if s.find(f"d_{ns}") >= -1:
    #     s = s.replace(f"d_{ns}", f"d_{status}")
    for p in ps_sels:
        if s.find(f"d_{p}.world") and p != sel:
            s = s.replace(f"d_{p}.world", f"d_{sel}.world")
            
    with open(file_location, 'w') as file:
        file.write(s)

if __name__ == "__main__":
    
    # Read the environment variables
    
    try:
        args = {
        "self_care_ability" : int(input("Enter the value for self_care_ability(0 or 1): ")),
        "alone" : int(input("Enter the value for alone(0 or 1): "))
        }
    except:
        raise ValueError()
    main(args)