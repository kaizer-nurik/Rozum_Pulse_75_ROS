from pyDHgripper.pyDHgripper import AG95
import time

gripper = AG95(port='COM3')


gripper.set_pos(val=0)

# while True:
#     gripper.set_pos(val=0)
#     time.sleep(1)
#     gripper.set_pos(val=1000)
#     time.sleep(1)

