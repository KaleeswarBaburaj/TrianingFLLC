
def run(robot):
    #robot.move_robot(200,200,150)
    robot.reset_heading()
    robot.gyro_straight(50, speed=500)
    robot.gyro_turn(45)
    robot.gyro_straight(375, speed = 500)
    robot.run_left_attachment(360,500,False)
    robot.gyro_straight(375, speed = -300)
    #robot.run_left_attachment(360,500,False)
    #robot.run_right_attachment(-180,600, False)
    # robot.stop_at_black_line(black_threshold=25)
    
    
    # robot.gyro_turn(90)
    # robot.gyro_straight(50)


