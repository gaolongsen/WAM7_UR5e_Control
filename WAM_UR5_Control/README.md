# WAM(7-DOF) and UR5e(6-DOF) for Python

[![A Python Robotics Package](https://raw.githubusercontent.com/petercorke/robotics-toolbox-python/master/.github/svg/py_collection.min.svg)](https://github.com/petercorke/robotics-toolbox-python)
[![Powered by Spatial Maths](https://raw.githubusercontent.com/petercorke/spatialmath-python/master/.github/svg/sm_powered.min.svg)](https://github.com/petercorke/spatialmath-python)[![PyPI version](https://badge.fury.io/py/roboticstoolbox-python.svg)](https://badge.fury.io/py/roboticstoolbox-python)
[![Anaconda version](https://anaconda.org/conda-forge/roboticstoolbox-python/badges/version.svg)](https://anaconda.org/conda-forge/roboticstoolbox-python)
![PyPI - Python Version](https://img.shields.io/pypi/pyversions/roboticstoolbox-python.svg)[![Build Status](https://github.com/petercorke/robotics-toolbox-python/workflows/Test/badge.svg?branch=master)](https://github.com/petercorke/robotics-toolbox-python/actions?query=workflow%3ATest)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

<table style="border:0px">
<tr style="border:0px">
<td style="border:0px">
<img src="https://raw.githubusercontent.com/gaolongsen/WAM7_UR5e_Control/main/WAM_UR5_Control/Pic/lobo.png" width="200"></td>
<td style="border:0px">
The project is for two robot arm—— WAM 7 Dof(with Barrett 282H robot hand) and UR5e 6 Dof(with 2 DoF gripper) working together in our Lab enviroment. Welcome anyone come up with any question and give us your avaliable corrections about that!
<ul>
<li><a href="https://github.com/gaolongsen/WAM7_UR5e_Control/tree/main/WAM_UR5_Control">GitHub repository </a></li>
<li><a href="https://petercorke.github.io/robotics-toolbox-python">Documentation</a></li>
</ul>
</td>
</tr>
</table>

<!-- <br> -->

## Contents

- [Before Getting going](#2)
- [Tutorials](#3)
- [Code Examples](#4)
- [Toolbox Research Applications](#5)
- [Toolbox ICRA Paper and Citation Info](#6)
- [Using the Toolbox in your Open Source Code?](#7)
- [Common Issues and Solutions](#8)

<br>

## Before Getting going

Our Project depends on Petercorke Lib - **Robotics Toolbox for Python** which is well known in robotics simulation. You need to make sure you have installed this package in your anaconda environment. We strongly recommend using python 3.8 to make sure all environment setups include anaconda, cuda and so on be the same as what we did to make sure your can run it successfully without any problem. All python >3.6 should also be fine.

### Install Mujoco, Anaconda, Cuda, Mujoco_py on your Ubuntu 20.04 system

*This part is only for our lab member tutorial.* We assume you know how to do that and all setup have been finished on your own PC. 



[Link for Google Doc tutorial.](https://docs.google.com/document/d/1OZ0ddXQztCEghgmXmJa1to4vAGpymkuLdKgb8dG8sik/edit?usp=sharing) 



### Using pip

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

### From GitHub

To install the bleeding-edge version from GitHub

```shell script
git clone https://github.com/petercorke/robotics-toolbox-python.git
cd robotics-toolbox-python
pip3 install -e .
```



### Environment in Mujoco

![](/home/agman-high/Dropbox/WAM7_UR5e_Control/WAM_UR5_Control/Pic/WAM.png)





