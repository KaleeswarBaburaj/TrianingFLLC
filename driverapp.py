from pybricks.tools import hub_menu
from cls_mainmission import MissionRobot

robot = MissionRobot() 

import drones
import seeds
import exploringseeds

missions = {"1": drones.run, "2": seeds.run, "3": exploringseeds.run , "4": seeds.run}

while True:
    selected = hub_menu(*missions.keys())
    missions[selected](robot)