# Author: Longsen Gao
# Contact: longsengao@gmail.com

from mujoco_py import MjViewer
import glfw

import numpy as np
import ikpy
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

open_viewers = []  # a static list to keep track of all viewers


class MjViewerExtended(MjViewer):
    """ An extension of mujoco-py's MjViewer. MjViewerExtended does not
        terminate all other viewers and the python interpreter when closeing.
    """

    def __init__(self, sim):
        glfw.init()  # make sure glfw is initialized
        super().__init__(sim)
        open_viewers.append(self)

    def close(self):
        """ Closes the viewers glfw window. To open a new one, create a new
            instance of MjViewerExtended
        """
        # MjViewer only calls glfw.terminate() here killing glfw entierly.
        if glfw.window_should_close(self.window):
            return
        try:
            glfw.set_window_should_close(self.window, 1)
            glfw.destroy_window(self.window)
        except Exception:
            pass

        open_viewers.remove(self)
        if len(open_viewers) == 0:
            glfw.terminate()

    def key_callback(self, window, key, scancode, action, mods):
        if action == glfw.RELEASE and key == glfw.KEY_ESCAPE:
            self.close()
        else:
            super().key_callback(window, key, scancode, action, mods)


class Wam4IK(Chain):
    """ A basic kinamatic model of the MjWAM4 """

    def __init__(self,
                 active_joints=[1, 1, 1, 1],
                 base_translation=[0, 0, 0.84],  # x, y, z
                 base_orientation=[0, 0, np.pi / 2],  # x, y, z
                 tool_translation=[0, 0, 0],
                 tool_orientation=[0, 0, 0]
                 ):
        links = [OriginLink(),
                 URDFLink(name="wam/links/base",
                          # translation of frame
                          origin_translation=base_translation,
                          origin_orientation=base_orientation,
                          rotation=[0, 0, 0]  # joint axis [0, 0, 0] -> no joint
                          ),
                 URDFLink(name="wam/links/shoulder_yaw",
                          origin_translation=[0, 0, 0.16],
                          origin_orientation=[0, 0, 0],
                          rotation=[0, 0, 1]
                          ),
                 URDFLink(name="wam/links/shoulder_pitch",
                          origin_translation=[0, 0, 0.186],
                          origin_orientation=[0, 0, 0],
                          rotation=[1, 0, 0]
                          ),
                 URDFLink(name="wam/links/shoulder_roll",
                          origin_translation=[0, 0, 0],
                          origin_orientation=[0, 0, 0],
                          rotation=[0, 0, 1]
                          ),
                 URDFLink(name="wam/links/upper_arm",
                          origin_translation=[0, -0.045, 0.550],
                          origin_orientation=[0, 0, 0],
                          rotation=[1, 0, 0]
                          ),
                 URDFLink(name="wam/links/tool_base_wo_plate",
                          origin_translation=[0, 0.045, 0.350],
                          origin_orientation=[0, 0, 0],
                          rotation=[0, 0, 0]
                          ),
                 URDFLink(name="wam/links/tool_base_w_plate",
                          origin_translation=[0, 0, 0.008],
                          origin_orientation=[0, 0, 0],
                          rotation=[0, 0, 0]
                          ),
                 URDFLink(name="wam/links/tool",
                          origin_translation=tool_translation,
                          origin_orientation=tool_orientation,
                          rotation=[0, 0, 0]
                          )
                 ]

        self.all_joints = [False, False, True, True, True, True, False, False, False]
        self.active_joints = list(map(lambda x: x == 1, active_joints))
        self.active_links = [False, False, *active_joints, False, False, False]
        Chain.__init__(self, name='wam4',
                       active_links_mask=self.active_links,
                       links=links)

    def fk(self, joints, full_kinematics=False):
        joints = np.array([0, 0, *joints, 0, 0, 0])
        return Chain.forward_kinematics(self, joints, full_kinematics)

    def ik(self, target_position=None, target_orientation=None, orientation_mode=None, **kwargs):
        full = Chain.inverse_kinematics(self, target_position, target_orientation, orientation_mode, **kwargs)
        active = self.joints_from_links(full)
        return active

    def joints_from_links(self, joints):
        return np.compress(self.all_joints, joints, axis=0)


class Finger1(Chain):
    def __init__(self,
                 # active_joints=[1, 1, 1, 1, 1, 1, 1, 1],
                 active_joints=[1, 1, 1],  # 3 joints
                 # base_translation=[-1.5, 0, 0],  # x, y, z
                 # base_orientation=[0, 0, np.pi / 2]  # yaw, pitch, raw
                 base_translation=[0., 0., 0.84],  # x, y, z
                 base_orientation=[0, 0, 0]  # yaw, pitch, raw
                 ):
        links = [OriginLink(),
                 URDFLink(name="wam/bhand/finger_1/prox_link",  # Without Track
                          origin_translation=[-0.025, 0, 0.0415],  # translation of frame
                          origin_orientation=[0, 0, -np.pi / 2],
                          translation=[0, 0, 1]
                          ),
                 URDFLink(name="wam/bhand/finger_1/med_link",  # Without Track
                          origin_translation=[0.05, 0, 0.0339],  # translation of frame
                          origin_orientation=[np.pi / 2, 0, 0],
                          translation=[0, 0, 1]
                          ),
                 URDFLink(name="wam/bhand/finger_1/dist_link",  # Without Track
                          origin_translation=[0.06994, 0.003, 0],  # translation of frame
                          origin_orientation=[0, 0, np.pi / 4],
                          translation=[0, 0, 1]
                          )
                 ]
        # Totally we have 3 links (original link + 3 links )
        # self.all_joints = [False, False, True, True, True, True, True, True, True, True]
        self.all_joints = [False, True, True, True]
        self.active_joints = list(map(lambda x: x == 1, active_joints))  # self.active_joints = [1, 1, 1, 1, 1, 1, 1, 1]
        self.active_links = [False, *self.active_joints]
        Chain.__init__(self, name='finger1',
                       active_links_mask=self.active_links,
                       links=links)

    def fk(self, joints, full_kinematics=False):
        joints = np.array([0, *joints])
        return Chain.forward_kinematics(self, joints, full_kinematics)

    def ik(self, target_position=None, target_orientation=None, orientation_mode=None, **kwargs):
        full = Chain.inverse_kinematics(self, target_position, target_orientation, orientation_mode, **kwargs)
        active = self.joints_from_links(full)
        return active

    def joints_from_links(self, joints):
        return np.compress(self.all_joints, joints, axis=0)


class Wam7IK(Chain):
    def __init__(self,
                 # active_joints=[1, 1, 1, 1, 1, 1, 1, 1],
                 active_joints=[1, 1, 1, 1, 1, 1, 1],  # 11 joints
                 # base_translation=[-1.5, 0, 0],  # x, y, z
                 # base_orientation=[0, 0, np.pi / 2]  # yaw, pitch, raw
                 base_translation=[0., 0., 0.84],  # x, y, z
                 base_orientation=[0, 0, 0]  # yaw, pitch, raw
                 ):
        links = [OriginLink(),
                 URDFLink(name="wam/links/shoulder_yaw",  # Start appear joint
                          origin_translation=[0, 0, 0.346],
                          origin_orientation=[0, 0, -np.pi / 2],
                          rotation=[0, 0, 1]),
                 URDFLink(name="wam/links/shoulder_pitch",
                          origin_translation=[0, 0, 0],
                          origin_orientation=[-np.pi / 2, 0, 0],
                          rotation=[0, 0, 1]),
                 URDFLink(name="wam/links/upper_arm",
                          origin_translation=[0, -0.505, 0],
                          origin_orientation=[np.pi / 2, 0, 0],
                          rotation=[0, 0, 1]),
                 URDFLink(name="wam/links/forearm",
                          origin_translation=[0.045, 0, 0.045],
                          origin_orientation=[-np.pi / 2, 0, 0],
                          rotation=[0, 0, 1]),
                 URDFLink(name="wam/links/wrist_yaw",
                          origin_translation=[-0.045, 0, 0],
                          origin_orientation=[np.pi / 2, 0, 0],
                          rotation=[0, 0, 1]),
                 URDFLink(name="wam/links/wrist_pitch",
                          origin_translation=[0, 0, 0.3],
                          origin_orientation=[-np.pi / 2, 0, 0],
                          rotation=[0, 0, 1]),
                 URDFLink(name="wam/links/wrist_palm",
                          origin_translation=[0, -0.06, 0],
                          origin_orientation=[np.pi / 2, 0, 0],
                          rotation=[0, 0, 1]),
                 URDFLink(name="wam/bhand/bhand_palm_link",
                          origin_translation=[0, 0, 0],
                          origin_orientation=[0, 0, -np.pi],
                          rotation=[0, 0, 1]),
                 URDFLink(name="wam/bhand/finger_3/med_link",
                          origin_translation=[0, 0.05, 0.0754],
                          origin_orientation=[np.pi / 2, np.pi / 2, 0],
                          rotation=[0, 0, 1]),
                 URDFLink(name="wam/bhand/finger_3/dist_link",
                          origin_translation=[0.06994, 0.003, 0],
                          origin_orientation=[0, 0, np.pi / 4],
                          rotation=[0, 0, 1]),
                 ]
        # Totally we have 10 links (original link + 9 links )
        # self.all_joints = [False, False, True, True, True, True, True, True, True, True]
        self.all_joints = [False, True, True, True, True, True, True, True, True, True, True, True]
        self.active_joints = list(map(lambda x: x == 1, active_joints))  # self.active_joints = [1, 1, 1, 1, 1, 1, 1, 1]
        self.active_links = [False, *self.active_joints]
        Chain.__init__(self, name='wam7',
                       active_links_mask=self.active_links,
                       links=links)

    def fk(self, joints, full_kinematics=False):
        joints = np.array([0, *joints])
        return Chain.forward_kinematics(self, joints, full_kinematics)

    def ik(self, target_position=None, target_orientation=None, orientation_mode=None, **kwargs):
        full = Chain.inverse_kinematics(self, target_position, target_orientation, orientation_mode, **kwargs)
        active = self.joints_from_links(full)
        return active

    def joints_from_links(self, joints):
        return np.compress(self.all_joints, joints, axis=0)
