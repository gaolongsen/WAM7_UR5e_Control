# Author: Longsen Gao
# Contact: longsengao@gmail.com

import re
import numpy as np


# r = "<key qpos='0.962 1.02 1.064 0.99974 -0.286 0 0 0 0 -3.13196 0.137401 -0.507761 0.563116 -0.11064 -0.175325'/>"

def WAM_UR5_Pose(pose):
    result = re.findall(".*'(.*)'.*", pose)
    MM = result[0].split()
    res = [eval(i) for i in MM]
    WAM_joint = np.array(res[:7])  # [:) left close, right open
    UR5_joint = np.array(res[9:15])  # [:) left close, right open
    return WAM_joint, UR5_joint
