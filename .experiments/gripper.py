from pyDHgripper import AG95
import time

gripper = AG95(port='/dev/robot/dh_ag95_gripper')


gripper.set_pos(val=0)

# while True:
#     gripper.set_pos(val=0)
#     time.sleep(1)
#     gripper.set_pos(val=1000)
#     time.sleep(1)

