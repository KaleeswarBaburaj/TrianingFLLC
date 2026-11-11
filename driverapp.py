from pybricks.tools import hub_menu
from cls_Main import MissionRobot

robot = MissionRobot()

import exploring_roots
import drones
import Katiedid

missions = {"1": exploring_roots.run, "2": drones.run, "3" : Katiedid.run} 

while True:
   selected = hub_menu(*missions.keys())
   missions[selected](robot)



