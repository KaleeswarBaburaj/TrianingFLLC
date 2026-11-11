from pybricks.tools import hub_menu
from cls.Main import MissionRobot

robot = MissionRobot()

import exploring_roots
import Drones
import exploringseeds

missions = {"1": opendoor.run, "2": Drones.run, "3": exploringseeds.run , "4": seeds.run}

while True:
    selected = hub_menu(*missions.keys())
    missions[selected](robot)

