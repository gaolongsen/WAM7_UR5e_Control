# WAM(7-DOF) and UR5e(6-DOF) for Python

[![A Python Robotics Package](https://raw.githubusercontent.com/petercorke/robotics-toolbox-python/master/.github/svg/py_collection.min.svg)](https://github.com/petercorke/robotics-toolbox-python)
[![Powered by Spatial Maths](https://raw.githubusercontent.com/petercorke/spatialmath-python/master/.github/svg/sm_powered.min.svg)](https://github.com/petercorke/spatialmath-python)[![PyPI version](https://badge.fury.io/py/roboticstoolbox-python.svg)](https://badge.fury.io/py/roboticstoolbox-python)
[![Anaconda version](https://anaconda.org/conda-forge/roboticstoolbox-python/badges/version.svg)](https://anaconda.org/conda-forge/roboticstoolbox-python)
![PyPI - Python Version](https://img.shields.io/pypi/pyversions/roboticstoolbox-python.svg)[![Build Status](https://github.com/petercorke/robotics-toolbox-python/workflows/Test/badge.svg?branch=master)](https://github.com/petercorke/robotics-toolbox-python/actions?query=workflow%3ATest)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

<table style="border:0px">
<tr style="border:0px">
<td style="border:0px">
<img src="https://raw.githubusercontent.com/gaolongsen/WAM7_UR5e_Control/main/WAM_UR5_Control/Pic/lobo.png" width="400"></td>
<td style="border:0px">
The project is for two robot arm—— WAM 7 Dof(with Barrett 282H robot hand) and UR5e 6 Dof(with 2 DoF gripper) working together in our Lab enviroment. Welcome anyone come up with any question and give us your avaliable corrections about that!
<ul>
<li><a href="https://github.com/gaolongsen/WAM7_UR5e_Control/tree/main/WAM_UR5_Control">GitHub repository </a></li>
<li><a href="https://petercorke.github.io/robotics-toolbox-python">Documentation</a></li>
</ul>
</td>
</tr>
</table><br>

## Before Getting going

Our Project depends on Petercorke Lib - **Robotics Toolbox for Python** which is well known in robotics simulation. You need to make sure you have installed this package in your anaconda environment. We strongly recommend using python 3.8 to make sure all environment setups include anaconda, cuda and so on be the same as what we did to make sure your can run it successfully without any problem. All python >3.6 should also be fine.

### Install Mujoco, Anaconda, Cuda, Mujoco_py on your Ubuntu 20.04 system

*This part is **only** for our lab member tutorial.* We assume you know how to do that and all setups have been finished on your own PC. 



[Link for Google Doc tutorial.](https://docs.google.com/document/d/1OZ0ddXQztCEghgmXmJa1to4vAGpymkuLdKgb8dG8sik/edit?usp=sharing) 

### Install Robotics Toolbox

#### Option 1: Using pip

**Note**: *Make sure you are in your anaconda environment with mujoco_py in your terminal!*

Install a snapshot from PyPI

```shell script
pip3 install roboticstoolbox-python
```

Available options are:

- `collision` install collision checking with [pybullet](https://pybullet.org)

Put the options in a comma separated list like

```shell script
pip3 install roboticstoolbox-python[optionlist]
```

[Swift](https://github.com/jhavl/swift), a web-based visualizer, is
installed as part of Robotics Toolbox.

#### Option 2: From GitHub

To install the bleeding-edge version from GitHub

```shell script
git clone https://github.com/petercorke/robotics-toolbox-python.git
cd robotics-toolbox-python
pip3 install -e .
```

### Modify the Robotics Toolbox package

For UR5 robot, you don't need to add anything for that due to the package has anything about. But for WAM, you need import WAM robot information to the toolbox package. We will contact with the the author of the toolbox and upload the WAM robot description to the package in the future. Go to this [**link**](https://github.com/gaolongsen/Package_Adding_WAM) to see how to import the WAM to the robotics toolbox package for python. 

### Environment in Mujoco

WAM Robot without Barrett BH282 hand installed 6 axis Torque/Force sensor(blue cylinder part)

<img src="https://raw.githubusercontent.com/gaolongsen/WAM7_UR5e_Control/main/WAM_UR5_Control/Pic/WAM.png">

Whole scene for WAM and UR5e on Mujoco.

<img src="https://raw.githubusercontent.com/gaolongsen/WAM7_UR5e_Control/main/WAM_UR5_Control/Pic/Whole.png">

Inertia of moment for all components in our environment include satellite model, solar panel model, hybrid hinge system, WAM and UR5. 

<img src="https://raw.githubusercontent.com/gaolongsen/WAM7_UR5e_Control/main/WAM_UR5_Control/Pic/IOM_Whole.png">

