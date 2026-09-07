# from cls_mainmission import MissionRobot

# robot = MissionRobot() 

#robot.run_left_attachment(100,0)
#robot.drive_base.turn(45)

def run(robot):
    #robot.move_robot(200, 200,150)
    robot.reset_heading()
    # robot.gyro_straight(500)
    # robot.gyro_turn(90)
    robot.run_left_attachment(360,500,False)
    robot.run_right_attachment(360,100, False)
    robot.stop_at_black_line(black_threshold=25)
    