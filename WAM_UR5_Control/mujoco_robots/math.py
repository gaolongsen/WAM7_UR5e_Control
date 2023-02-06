# Author: Longsen Gao
# Contact: longsengao@gmail.com

import numpy as np

def transform_pos(trafo, pos):
    if pos.ndim == 1:
        assert pos.size == 3
        ret_pos = trafo[:3, :3] @ pos + trafo[:3, 3]
    else:
        assert pos.shape[1] == 3
        ret_pos = np.zeros_like(pos)
        for i in range(pos.shape[0]):
            ret_pos[i, :] = trafo[:3, :3] @ pos[i, :] + trafo[:3, 3]
    return ret_pos


def transform_vel(trafo, vel):
    if vel.ndim == 1:
        assert vel.size == 3
        ret_vel = trafo[:3, :3] @ vel
    else:
        assert vel.shape[1] == 3
        ret_vel = np.zeros_like(vel)
        for i in range(vel.shape[0]):
            ret_vel[i, :] = trafo[:3, :3] @ vel[i, :] + trafo[:3, 3]
    return ret_vel


def transform_jac(trafo, jac):
    if jac.ndim == 2:
        assert jac.shape[0] == 3  # position only
        ret_jac = trafo[:3, :3] @ jac
    elif jac.ndim == 3:
        assert jac.shape[1] == 3  # position only
        ret_jac = np.zeros_like(jac)
        for i in range(jac.shape[0]):
            ret_jac[i] = trafo[:3, :3] @ jac[i, :]
    else:
        raise ValueError("jac has to be 2 or 3 dimensional")
    return ret_jac


def transform(trafo, pos=None, vel=None, jac=None):
    ret = []
    if pos is not None:
        ret.append(transform_pos(trafo, pos))
    if vel is not None:
        ret.append(transform_vel(trafo, vel))
    if jac is not None:
        ret.append(transform_jac(trafo, jac))
    if len(ret) == 1:
        return ret[0]
    else:
        return ret
