from pprint import pprint
from pulseapi import *
from time import sleep
from pyDHgripper.pyDHgripper import AG95

host = "10.10.10.20:8081"
robot = RobotPulse(host)

SPEED = 10

gripper = AG95(port='COM6')

def close_gripper():
    gripper.set_pos(val=0)
    sleep(2)
    
def open_gripper():
    gripper.set_pos(val=1000)
    sleep(2)


try:
    while True:
        open_gripper()
        # robot.set_position(position([-0.183, -0.115, 0.638], [-3.054326295852661, 0.0174532923847437, 3.1066861152648926]), SPEED, MT_JOINT)
        robot.set_pose(pose([1.9098739624023438, -88.22776794433594, 0.92767333984375, -1.6649017333984375, -95.06881713867188, -89.83932495117188]), SPEED)
        robot.await_motion()
        # robot.set_position(position([-0.184, -0.115, 0.638], [-3.054326295852661, 0.0174532923847437, 3.1066861152648926]), SPEED, MT_JOINT)
        robot.set_pose(pose([1.9126205444335938, -88.24150085449219, 1.162506103515625, -1.617523193359375, -95.05439758300781, -89.84207153320312]), SPEED)
        robot.await_motion()
        # robot.set_position(position([-0.184, -0.115, 0.637], [-3.054326295852661, 0.0174532923847437, 3.1066861152648926]), SPEED, MT_JOINT)
        robot.set_pose(pose([1.9002609252929688, -88.20991516113281, 1.1961669921875, -1.6182098388671875, -95.08872985839844, -89.84001159667969]), SPEED)
        robot.await_motion()
        # robot.set_position(position([-0.344, -0.121, 0.26], [-3.0194196701049805, 0, 3.0892326831817627]), SPEED, MT_JOINT)
        robot.set_pose(pose([2.8231124877929688, -100.118408203125, 127.1358642578125, -117.00213623046875, -97.30316162109375, -89.82009887695312]), SPEED)
        robot.await_motion()
        open_gripper()
        # robot.set_position(position([-0.357, -0.099, 0.19], [-3.1066861152648926, 0.0523598790168762, 3.1241393089294434]), SPEED, MT_JOINT)
        robot.set_pose(pose([-2.5135546535782964, -88.82815191120602, 133.57655538870003, -131.8369731835438, -92.12692516130835, -93.40712383282602]), SPEED)
        robot.await_motion()
        close_gripper()
        # robot.set_position(position([-0.357, -0.099, 0.25], [-3.1066861152648926, 0.0523598790168762, 3.1241393089294434]), SPEED, MT_JOINT)
        robot.set_pose(pose([-2.5135546535782964, -98.03956122682314, 125.59745704138376, -114.64646552061043, -92.12692516130835, -93.40712383282602]), SPEED)
        robot.await_motion()
        # robot.set_position(position([-0.357, -0.099, 0.5], [-3.1241393089294434, 0, 3.1066861152648926]), SPEED, MT_JOINT)
        robot.set_pose(pose([-2.896814794698173, -99.28940104757709, 71.25904367622567, -62.02018378260749, -90.99872502230937, -94.89725263647898]), SPEED)
        robot.await_motion()
        # robot.set_position(position([-0.357, 0.099, 0.5], [-3.1415927410125732, 0, 3.1415927410125732]), SPEED, MT_JOINT)
        robot.set_pose(pose([-34.23547043097972, -99.3838539107727, 71.29281357001051, -61.908955305988, -89.99999585893323, -124.23546542202342]), SPEED)
        robot.await_motion()
        # robot.set_position(position([-0.357, -0.099, 0.4], [-3.1415927410125732, 0, 3.1415927410125732]), SPEED, MT_JOINT)
        robot.set_pose(pose([-3.236919971113707, -105.34010254060115, 98.98277655027708, -83.64267219161138, -89.99999499903504, -93.23691496215736]), SPEED)
        robot.await_motion()
        # robot.set_position(position([-0.357, -0.099, 0.25], [-3.1415927410125732, 0, 3.1415927410125732]), SPEED, MT_JOINT)
        robot.set_pose(pose([-3.236919971113707, -98.7547576406115, 128.2642901566266, -119.5095306979506, -89.99999499903504, -93.23691496215736]), SPEED)
        robot.await_motion()
        # robot.set_position(position([-0.345, -0.09, 0.19], [-3.1415927410125732, 0, 3.1415927410125732]), SPEED, MT_JOINT)
        robot.set_pose(pose([-4.876148428574368, -91.62194515623708, 138.9894521824422, -137.36750506519857, -89.99999500917221, -94.87614341961807]), SPEED)
        robot.await_motion()
        open_gripper()
        # robot.set_position(position([-0.345, -0.09, 0.22], [-3.1415927410125732, 0, 3.1415927410125732]), SPEED, MT_JOINT)
        robot.set_pose(pose([-4.876148428574368, -97.5946194816683, 135.26514697352857, -127.67052553085378, -89.99999500917221, -94.87614341961807]), SPEED)
        robot.await_motion()
        # robot.set_position(position([-0.183, -0.115, 0.638], [-3.1415927410125732, 0, 3.1415927410125732]), SPEED, MT_JOINT)
        robot.set_pose(pose([-1.261194376140736, -94.67038518473646, 12.542796757608901, -7.872409927389754, -89.99999499225714, -91.26118936718444]), SPEED)
        robot.await_motion()
        
        break
except RestApiException as e:
    pprint("Exception when calling RestRobot %s\n" % e)
