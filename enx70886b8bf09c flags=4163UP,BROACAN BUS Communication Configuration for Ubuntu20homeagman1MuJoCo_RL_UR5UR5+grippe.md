1. **enx70886b8bf09c: flags=4163<UP,BROACAN BUS Communication Configuration for Ubuntu20/home/agman1/MuJoCo_RL_UR5/UR5+gripper**

**Problem:** when we start to build up communication between WAM computer(system installed with Ubuntu20) and Barrett robot arm, we may meet this problem that the CAN port can’t be found. Such as the shot screen below.

![img](https://lh7-us.googleusercontent.com/8Vr967lX_wK25IhuJtu5bcmP2Abl5058QyGnwgZM6uD8mJhLinstcN4lQvg023msKErZqIjNGO-JxueI4EliF-_MzfYYejzv36P3EpuDpzcvzbo14RCEV-Sx8EC6Ug6LyM93SKOJD5Ed1Hap_5HPHA)

**Solution:** send the command on a new terminal: 

$ sudo ip link set can0 up type can restart-ms 100 bitrate 1000000

Then reboot the computer, send the command: 

` cd catkin_ws`

` roslaunch wam_node wam_node.launch`

￼

Then follow the notification step by step.

1. Press the “Shift” + “Reset/Idle” button at the same time

![Image](https://lh7-us.googleusercontent.com/rPWQWqEd3yFEmLa0crPBgJjI8cyMfCYS2dyaMeHPoIM3JD1N00hKDO6XrIOwsYloc0UTwB2seHCHyKl-4kMMuMrXMxd-SUZHoTBZJC-Kfy6Udk5N6NFP_WtAuNT-toXldmtFi5U1Hop4VryUAdA0EQ)

1. When the “Reset/Idle” button light. Press “Enter” on the keyboard as the follow of notification from the terminal:

![Image](https://lh7-us.googleusercontent.com/zuZK7UUP8fX9EgC8dETWUH9WxOMdi7pGV5BpVz2hyhyjcKbZS0IdvZVJn2Sk7B2g9rkD1kQTvGwAUvQ-8z9jWHhHWkP5Y-n0niA0nXgKwfhhQFZmYvUwFaOZljG6oElXlzeWeteEEyJEAL78r7BTVA)

![img](https://lh7-us.googleusercontent.com/DH9DKsIxFUmq_cvrGx9GuHG4MMdD1ff87BF8u7ZBiLN7crl3lFoWhLIArl3YRJiT_M75pAHBqBUuRKq-hmC8m5DEC1PFSp-aDaTIre5cxL6UdfmyRQPdc4UingUT8tTdhI8z-REYFqQ9o2Md61IW1Q)

1. Press the “Shift” + “Activate” button at the same time. When the “Activate” button lights, the communication can be built up successfully.

![Image](https://lh7-us.googleusercontent.com/EBwXWWA-kz-RmWuwp-xhOCNJkc4qneanehLntbIWXNs6qGwDJqwmXJ_6v9fifdReMF_457FodYN9JHwEdoIYOXVU-E9KuE7bFGuUuuvVkHSVD5qhUDPJG5TKlzsUdwCUtFukbd02TFRV5U0_WSxKjQ)

1. **Communication problem for CAN - USB adapter** 

When you need another computer(not WAM PC) to control the Barrett Arm or hand, you need to use a CAN-USB adapter to build up communication between your PC and WAM robot unless your computer is equipped with a PCI card. The instructions for installing the related drives, libraries, and packages can be found in the links below:

Install libbarrett and ROS Noetic on Ubuntu 20.04:

https://git.barrett.com/software/barrett-ros-pkg#on-ubuntu-2004

Install barrett-ros-pkg:

https://git.barrett.com/software/barrett-ros-pkg#compiling-the-package

After you install everything needed and make sure your system can recognize the CAN port, such as the picture shown below(you can find ‘pcan32’ ‘pcan-usb’ and ‘pcanusb32’):

![img](https://lh7-us.googleusercontent.com/ClNdAWKZc2jaN3jVwnFzdXS0tdyho9_j6tiR1iIFFcI2NjAXsUIMg0Q9JP9hzn7mwQsrUdVHGbJYIoKY3hhpE9ma5BSqdcKAZIh95qnEJSJcOZWD1yeyTo-gpVL1a-LPPVaDQVuIWa-x_Gu4Z1LDBg)

Then you can open a new terminal and input the command

” roslaunch wam_node wam_node.launch”

If the node can run successfully, that’s good; if not, you need to change the delay time in puck.h file:

file path: gedit /home/wam/libbarrett/include/barrett/products/puck.h 

 *(ps: XXX is the name for the administrator of your system)*

![img](https://lh7-us.googleusercontent.com/6DgSif69ub6rkA-991lPIu_JSx_hdsz2SjoURl7EqJc1B_Njuc0ZnmxyPkbCrY4KN0I-d-EndG9x1cFLTb2gXReXJAp42yupZkryZVy7QmKL02_BrKFFn3CcygMn8quMKWflgbfbOZjM_g-kFyG2Lw)

Change the time-delay variable: **timeout_s** in lines 118 and 121 larger than 0.001 (Here, I changed it to 0.005, it depends on the performance of your PC, the higher the performance of your computer, the longer the delay-time variable should be.)

After that, save the file and rebuild the drivers and libraries with the followed instructions:

`cd libbarrett`

`cmake .`

`make`

`sudo make install`

`cd ~/catkin_wam/src/barrett-ros-pkg`

`sudo -s`

`./build.sh`

After that, you can control the Barrett robot on your own computer. That’s all. 

**3. Problem and correct related operations for Vicon system in Agman Lab**

Sometimes when we decide to make the Initialization for the Vicon system and reset the cartesian coordinate system, we need to use *ViconTracker**(now the newest version is 3.9)* to make the configuration for the Vicon system. Firstly we should open the software, then find the page as below:

![img](https://lh7-us.googleusercontent.com/guQMyzfiFFEidOD_W2bzCrb59u_urRD4PJFuWKB8WvQplVQBRHibzDFd8Ez2M5a5tSE7PUvV3gHKa2objiDwV-Q_u9_sARbVo4mMgFOXgcFyOpgc6XfOrhB-XMbzaGg39YMNeLSuGyRx2h8ugx6itAI)

​	

1. Camera Selection
2. View Changer
3. Important tabs
4. Non-functional Camera symbol

When connecting the Vicon system, you must be sure that the cameras you are interested in working with are all pointing in the direction of the scene you are working in. If the cameras are pointing in the wrong direction, you will see a symbol like (4) in the image above, they might show up with nothing in the frame when looking at camera view similar to the image below.

![img](https://lh7-us.googleusercontent.com/tUTG3anVXtUCb7xGiE0N4G56nmxRIg9le9piB6yxtayx2FtpxFluAkHitDLn6RU8zcqx9zCcVU-QGLAvbDwXTtfFJaCG9-dJIMRcfOtWq2OdmCYlEW4cnVfOaFPMRb0olBdskjj8JBIPxSMoG2q6xPg)

If you run into the scenario mentioned above, the first thing you should do would be to double check your camera positioning to make sure the camera is facing in the right direction and is even capturing the markers you’re interested in.

If you find that the camera is capturing the correct frame, then another option would be to lower the “Threshold” option under that camera's specific properties.

![img](https://lh7-us.googleusercontent.com/5axSf32unTO3kdp1wE6-QpBURiv4lsjkRdtutTRNrK8oygbGUMZgAPGfiG0eS8NAnZuGojrpm083F5MJ6XIfF7rDDJzWzfab2NhSqBTRT6QoAWLi_Po8_jzrnnuJ05S40imRkc1PQmMAuXJ0UcU5IRM)

Once the threshold has been adjusted, your camera should begin showing marker positions, and then you’re ready for the next step.

![img](https://lh7-us.googleusercontent.com/0c7-0e7T5VGzDqnh-BfYFUH0DcRdECGfhM2swyDbrrqBoryaWhDshE-cZ_p_j0mXw3KBAh4hfQiHxbe8Vqyku9UsBahX7IQqI0_jCutAAd7p1m0PUlHnBvW3Tin8ywpDk1z-4PoUe1kxHwqu3_EoGw)























①. Click the ‘ **Calibrate**’ button

②. Find the search column for ‘ **Camera Calibration Feedback,’** then choose the cameras which you need to initialize.

③. After finishing the choice, click the ‘ **Start** ’ button in the ‘Calibrate Cameras’ column.

/*********************************************************************************************

**Notice**: Here, we may meet the error after we click the ‘start ’ button as the situation happened below![img](https://lh7-us.googleusercontent.com/P1zeSZZVOB_JoMAk0QGg6VczZcItyvdEbzqkrgPYwtJiMtQCFg5Dog5Fwj_LnZOElGG9MwJ5tMVBmrQVqtC8WawGX19DCqPlkd-vH-zOzBfg205xB4fsh-bnPhJVyBnYJ8Cq2Hs1_LMhxlmWb4xKNg)

As the terminal reported, the Tracker is unable to write to the calibration x2d file. For this error, we need to find the path of data storage for Tracker:

![img](https://lh7-us.googleusercontent.com/REnVdMqbcIUm9foXQDmlGYAdnT4-HD0c6RxU3yw2FiJydWjwzo5qIIi1VJt9dvDI-jO6TpO2DnD7CzvSsv57CLMnR8DK-wcXt8mXd23v076zXPDTQAq3Js-q68mTS-f4djo4py_8emKNzFV15FLt5A)

Here we can find that there is a compressed file and other previous data files included .x2d file. 

Firstly, we need to move all files except the last one to the compressed file to save the previous data(if required). This operation makes sure that there are no .x2d files and .xcp files in this path to let the tracker write new .x2d and .xcp files here. 

Secondly, we back the tracker, do the same operation as we mentioned above from ①~③, 

Then we can initialize the vicon system successfully, as the shot screen below:

![img](https://lh7-us.googleusercontent.com/kXz-oZDFId5ohmuqgxPZKeQ8kh1x620DrXqu3sStFXhP39HmhKfQNLJTvTEpCsCU6NkI7NPeGrIkk-b7d6lLqfkMy6mNDLIASuL1_8uR0A7xlhrLXpQKp6wBVbIZW_HWPvAQF-4xX0XfUfO_MORBBw)

************************************************************************************************/

④. After we initialize the cameras we selected successfully, we need to set the origin point for the 3-D workspace to build up a new Spatial Cartesian Coordinate System. Here we choose one position, which should be the center point of the workspace’s base on the ground.

![img](https://lh7-us.googleusercontent.com/brk8wDc6Zxs4_LXLxS-JvyK_cK_nZr05AZeIm2CEWX-RJM0fgnuxjgcok1yLle4QQF7O8M2qBEpm_kr5WEtfJPiSaCBZBwGjwg5FuwXC9qo6ac39BcBGMJyJ7LSj17pRKpFGqMTt7YVjpYhrjGdykw)

Next, we find the ‘ **Set Volume Origin** ’ column on the left and click ‘ **Start**.’ Then we can see the originality part as below:

![img](https://lh7-us.googleusercontent.com/PJg8qzWxeZUdA_v6AYCt4HY-weud4mcxLhJ2fg0LqVScjYTAGJ1OkqmaJb5Js-pPXkrxcVZDgHwk51hYJhoNEzlHaGn462uLHdML2Zj9SMhpTfZ3zcLsXCOukSQLFKSarKavSy81EejqeV-6Mer8XQ)

⑤. After we click the ‘ **Set Origin** ’ button, all the positions for cameras will be reset, and the new spatial cartesian coordinate system has been built up. 

![img](https://lh7-us.googleusercontent.com/No5bSLqPfxWkCL64mNV3iwBmXTj7hyPYH6jAiDgegOIv99hMarqI6__Y_sUS8_3R_FF1eIG58I2CA9t5zJrxOz7yvx0FD81IgKYj5HXs70Kr-xRdjmhJlMmsLxg0YEZw3d5Q6WkyIUf6P4ov63oi0Q)

Once you’ve collected your data using the recording section, you can view the graphs associated with your trial by loading in a trial, 

 

![img](https://lh7-us.googleusercontent.com/v2qJp5vXZaNxzYOMfEqOwAIeTBhj4fWXNeHYZSnONNCAcAu_VZff-ZahdCatIzlJ1rfWre5HeBce1yMUcpJ19_NyyFk2MQ-lPQmtztb2QSHiaBsnugaVmo4RxW83a1u8zCxJqlKqRg3KL84jsnElbfo)

selecting the object in the tracked object in the 3D perspective view, and then switching to graph mode. Once done, you should select the appropriate graph using the drop-down menu, similar to the image below.

![img](https://lh7-us.googleusercontent.com/QHzJmLQ43GKczmpU2-y69gTKcg5LlB-3c9cZt0oa35eCeLhQK0iM2Ckg82PcqA1Tmm_9q6UJfZqz5RhM8VKlmG8UNnwByZzdfVKrpx4zAoemmJCM18AblpYPyWf3yf_3myiz-F6F4wZm5JBRcaNZCgE)

You can then view the position graphs, and from there you can select the velocity and acceleration graphs.

![img](https://lh7-us.googleusercontent.com/zkbcjik-tqDAtnrtUdnLhtyT_HgfA9Va6xK1nbEGnA_9o-YyTW02DlMODGCHRXK-6LCL98XC7vpA4FrdcFIzK--p-O4VdupG4C3L0IaGZdE9IgfZfUG8DmvGhcUNhVyfdXoe3bQTLEfitOusL2IyMBw)

**The problem for C++ or python scripts crushed when we are controlling WAM** 

**![img](https://lh7-us.googleusercontent.com/rEFWLjduPbRp0JAHTp3QwUYECz5SOZ2FY9PArqKDiyATYlcLesNUQXEANSFroKjIxj1t9HZIwE_VhFOYHmdoqK3DMzD7LHAaraz2RgcWwlEGdqZr5cuDgA7b7BLFvw2VwRXcT9D2BAtkdDXTMUDalw)**

Sometimes when we are building up communication with the WAM robot arm and controlling it, we may meet problems such as the figure above, the problem that the C++ or Python scripts you programmed would be crushed when you decide to run them. For this problem, firstly, make sure that the script you programmed is correct and whether the hardware settings in the program follow precisely the parameters set in the manual. 

Secondly, If everything is okay, then you can check out the file as the following path: 

libbarrett/include/barrett/bus/bus_manager.h:112

If your computer is high-performance, you should decrease the variable ‘MESSAGE_BUFFER_SIZE‘ less than the primitive value. Or, if your computer is in low-performance, you need to increase the variable larger than the primitive value. For my situation, I decreased the value for that variable and made the script run successfully on my high-performance computer. Here is my setting as below:

![img](https://lh7-us.googleusercontent.com/exDxPCPfl0ETJdYCABj02WVi9bx7I7EmHEXu0Sb0JQMq6MieMFVVKHF47Uglkxebcgx5Ts2ZG9QGCDqQ1czPB6DkGPJHICC4S5iueZie1NhMpQhlbFtAihQ0U3114Fwm9VQOX5kFAqG0b1JbjSSV5w)

After that, you need to rebuild & re-install the libbarrett library as we recorded in the ‘**Communication problem for CAN - USB adapter****.’** Then you can run the script programmed by yourself successfully. 

**Mujoco and mujoco-py installation instruction(Python 3.X Version)**

(Note: Don't install other versions with other python versions!)

*-Last update: 2022.6.8*

**Step 1: Install anaconda**

[Anaconda3-2021.04-Linux-x86_64.sh](https://repo.anaconda.com/archive/Anaconda3-2021.04-Linux-x86_64.sh)

​	`sudo chmod +x Anaconda3-2021.04-Linux-x86_64.sh`

​	`./Anaconda3-2021.04-Linux-x86_64.sh`

**Step 2 : install git**

​	`sudo apt install git`

**Step 3 : install the mujoco library**

1. Download the Mujoco library from 

​	https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz

2. create a hidden folder :

 	`sudo mkdir /home/username/.mujoco`

​	(Note: if the folder you created without any permission, use “**sudo chmod a+rwx   /path/to/file**” to give the folder permission)

3. extract the library to the .mujoco folder
4. include these lines in .bashrc file(terminal command: **gedit ~/.bashrc**):

`export LD_LIBRARY_PATH=/home/user_name/.mujoco/mujoco210/bin`

`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia`

`export PATH="$LD_LIBRARY_PATH:$PATH"`

`export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so`

5. source ~/.bashrc
6. Test that the library is installed by going into:

 		cd ~/.mujoco/mujoco210/bin

​		./simulate ../model/humanoid.xml

**Step 4 Install mujoco-py:**

​	`conda create --name mujoco_py python=3.8`

​	`conda activate mujoco_py`

​	`sudo apt update`

​	`sudo apt-get install patchelf`

​	`sudo apt-get install python3-dev build-essential libssl-dev libffi-dev libxml2-dev` 

​	`sudo apt-get install libxslt1-dev zlib1g-dev libglew1.5 libglew-dev python3-pip`

​	`git clone https://github.com/openai/mujoco-py`

​	`cd mujoco-py`

​	`pip install -r requirements.txt`

​	`pip install -r requirements.dev.txt`

​	`pip3 install -e . --no-cache`

**Step 5** **reboot your machine**

**Step 6** **run these commands**

​	`conda activate mujoco_py`

​	`sudo apt install libosmescd examplesa6-dev libgl1-mesa-glx libglfw3`

​	`sudo apt-get install libosmesa6-dev`

​	`sudo ln -s /usr/lib/x86_64-linux-gnu/libGL.so.1` 

​	`sudo ln -s /usr/lib/x86_64-linux-gnu/libGL.so`

​	`pip3 install -U 'mujoco-py<2.2,>=2.1'`

​	`cd examples`

​	`python3 setting_state.py`

**If you’re getting a Cython error, try:**

`pip install "cython<3"`

**MSI Laptop Nvidia Driver problem for multi-monitors display solution:**

[**Disabling secure boot** in BIOS options.](https://www.gigabyte.com/us/Support/FAQ/3001)

*(ps: also same solution for MSI Laptop with PCAN detection problem-No found pcan-32)*

[**CUDA Install Problem Solution Reference.**](https://askubuntu.com/questions/940582/upgrade-or-uninstall-cuda-to-allow-apt-get-to-work) **(Purely delete Cuda packages and Nvidia drivers on Ubuntu)**

`dpkg -l | grep cuda- | awk '{print $2}' | xargs -n1 sudo dpkg --purge`

`df -h`

`sudo apt-get purge nvidia\*`

`sudo apt-get -f install`

`sudo apt autoremove`

**Mujoco Environment setup in PyCharm:**

**![img](https://lh7-us.googleusercontent.com/z4L-t2sVpGKvzHiWJxMwDbKDmy-5mtNwrQOMlzub4RdkKpX8il_EbX_TYsbevcASZowu4XWHkaNLTfe4WIjHS2T6MNa7NISmbrN0hjZv5lZEMktgsLQD_Fw9-6oWTERoh0Lgg7fm8JNprcrsatZrvLQ)**

**Environment variables:** LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/wam/.mujoco/mujoco210/bin:/usr/lib/nvidia;LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so

**Note****: Strongly recommend all when you try to install mujoco through** **pip install mujoco****, keep in mind that the newest version is change a lot for its Dataset lib, that means when you run your previous projects, the error may occur and it’s hard to change for every data structure, so please use** **pip install mujoco==2.3.0** 

 you can ref this [link](https://github.com/Farama-Foundation/Gymnasium/issues/749)



**UR5e Operation Information:**

1. **Activating the Robot**
2. **Activating the Gripper**
3. **Activating the Wrist Camera**
4. **Programming the UR5e via Python**

1. **Activating the Robot:**

To begin using the robot, one must first ensure the controller is plugged in, and the connections to the controller are valid.

To turn on the UR5e, you hit the power button located on the front of the teach pendant next to the emergency stop button.

![img](https://lh7-us.googleusercontent.com/SF-hAmkxadMSAK3buIaacUlW7hPkr-2ZIyWd143PgJbT7HMgOJtmDwb9eNAEQ0gQ75HK91N8G1FsBmwRYmGHfdMsVab6p8E-A9OA9qVO1PDbpU3UDwWNvLVn7Dbh5AtZxPtqRvmxh5euy-zsd2L4Iw8)

Once the system has turned on, you still must activate the robot by selecting the button located at the bottom left corner of the screen. 

Once clicked, you will be taken to this screen. Press the “ON” button to activate the robot.

After providing power to the robot, click the “START” button to unlock the robot’s joints.

![img](https://lh7-us.googleusercontent.com/XqCK__JrxdwNvPdZzV5MvYrJdXRoHG0mWuSFGiG5gxxYpQnmu9rH0OzcavkDBPPoEixTFLOvhOzYjpI2G_EegTA_690vEQdssCIe3G5v4hjQxQHM54cAC7kl2rFuvosUqwnlUqWpxPXkdqKEt3PEkv8)

After being unlocked, if everything went well, your screen should look as it does below.

![img](https://lh7-us.googleusercontent.com/7PKBR2K7ZgmS6i5G_fHvttFOhuy_8fDtd_akOLU24rOj5B4S-6qPwIoW7lYetb2BwiiEUfz9QqabOQ0snaKomzl651ZouUajoq174qJOrZbd8mXXFf_YQm1Zi0mR9YOzfr5-5scRX9_X1BDX0jmD5dw)

If your screen looks like the one above, hit the “Exit” button.

With the robot active, you can begin moving it easily using the “Move” tab. An example of what your screen should look like is below.

![img](https://lh7-us.googleusercontent.com/pI5rSs_En9A0XsLnOEoDhdLfF6DwKezMrWSLjU8IyK1vRpjIivwDGSthtXanvsIA9kLxlamMrqiwFNZWsgebBcPw5Nr0iDpT_WvUFWSgW1ea9rDxL0r8T9fPkEUvRSWIYeAIFkCkZfiC-5C_XbNBiTs)

1. **Activating the Gripper**

With the robot on and activated, you can now activate the gripper supplied by RobotIQ.

**Gripper Model:**

RobotIQ 2F-85

![img](https://lh7-us.googleusercontent.com/rRf471l7KaKsR36Tojb77ksJ1n7EXPT6uYSc_dOwnOs194eh7voMNJNFhLy8uQ7RhPWTK66c9WzFopz3mybHqal0HHOX_dv1Z4yYmvHt4jAeez6xAlwgvNjkKizbmPLpb-QqsgbWDOYQ9fSl63FOZLU)

The gripper connects to the UR5e via a UR Cap, which is a softwa

![img](https://lh7-us.googleusercontent.com/xQnLV000sC6MnagJDesrk9mPfThjNf0PP5KMEGN1bZR5vkjP0OLp_uh7OhtuHlrUZVEHF03PJRl3rj-UPTD5jnnBXg1Lkzgnhy-_h8rRGT6xZ-FGsCP2lVA-WwaqssTGPq7Z1omzS6nNeGOSOTkCUmc)



1. **Activating the Wrist Camera**

**
**Product Website: https://robotiq.com/products/2f85-140-adaptive-robot-gripper?ref=nav_product_new_button 

![img](https://lh7-us.googleusercontent.com/IST0vqLihj47w25QUW1Ujmvc8wUNJ5JhI_Dmzy7t6xsnhHlcL032pZeQr1fe366rgnbNpVUwHIh9LOd02IcK5c4UrNJg-7MIWB5EkjWuHcEsHE31QvdUXQd0ucA3wP11YV4K4hC45icopp5VSSPVVVk)

1. **Programming the UR5e via Python**

You will need either Ubuntu 18.04 with ROS melodic, or Ubuntu 20.04 with ROS noetic. We are all using ROS Noetic with Ubuntu 20.04 at our lab.

**Drivers & Prerequisites:**

First, begin by installing the Universal Robots ROS Driver here: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver 

For the UR5e to function properly, you must install the External Control URcap, **which has already been installed on our UR5e,** but installation information can be found here: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_e_series.md 

You will also need to set up the tool communication with the UR5e if you would like to access the RobotIQ gripper. Information how to install this URcap can be found here, **which has been installed on the robot already**: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/setup_tool_communication.md 

Each UR robot is calibrated inside the factory giving exact forward and inverse kinematics. To also make use of this in ROS, you first have to extract the calibration information from the robot. Information on how to do so can be found in the Universal Robots ROS Driver in the first link. If we ever get another robot, it would be a good idea to follow the details on this page:https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_calibration/README.md 

Once you have the drivers setup, use this script to collect the calibration information from the robot. First, make sure the robot is turned on and connected to the network.

`conda deactivate`

`$ roslaunch ur_calibration calibration_correction.launch \`

 `robot_ip:=192.168.1.17`

 `target_filename:="${HOME}/my_robot_calibration.yaml”`

Once you’ve done this, you should find a file named my_robot_calibration.yaml in your home directory. Just leave that there.

Next, open another terminal, and input the following to set-up a connection with the robot:

`$ roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.1.17 kinematics_config:=${HOME}/my_robot_calibration.yaml`

If you’re getting an error, it is likely the driver was installed incorrectly or you have not properly set-up your root file. To alter your root file, use:

​	`gedit ~/.bashrc`

This is what my root file looks like:

![img](https://lh7-us.googleusercontent.com/xRGiO-XSniFMZ-yXcdbhHyGX-WBAUCTF9JngYJuadK-M0Gq4f7Y-cy-YL9xr5UQjYY3Be7-CHPQN_IzdWcmphRSBbHxfnBeZ8vHnlnJYpsiZmmrCzqUNyXL6e0q7MT8rHN0i95jB8ve8erEfSWFLp2I)

**Setting up the RT Kernal (NEED TO FINISH)**

To operate the UR driver it is recommended to setup a Ubuntu system with real-time capabilities. Because we are using an e-series UR, the higher control frequency might lead to non-smooth trajectories if we don’t run the system using a real time-enabled system.

To do this, I followed the following tutorial: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/real_time.md 

There were a few things that did not get explained in the tutorial, so I will elaborate here.

Make sure to check what kernel version you have installed in your system, and use this version instead of the 4.14.139-rt66 version they were using.

I found it helpful in the *“Getting the sources for a real-time kernel”* section to paste the HTML provided in their example into a browser and use that to find the appropriate file needed for the *wget* action.

In the Setup user privileges to use real-time scheduling section, to edit your /etc/security/limits.conf file I used

sudo nano /etc/security/limits.conf

to add the @realtime lines in the tutorial. Then you can use *Ctrl + O* to save the file.

**HAVING ISSUES WITH THE Setup GRUB to always boot the real-time kernel SECTION**

My rt kernel doesn’t appear to show up as a choice when doing their

`$ awk -F\' '/menuentry |submenu / {print $1 $2}' /boot/grub/grub.cfg`

**I’ll come back to this (^^^) later.**

**Using RVIZ and MoveIt!**

This website has great information on how to get rviz & MoveIt! up and running: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md 

I was able to connect the UR5e to my computer using the instructions followed in their tutorial, but I was having issues once I started trying to implement MoveIt!. It is likely I need to look at my current installation and make adjustments.

The UR External Control must be playing for the robot to be able to accept commands. You should be able to connect to the robot and see its position in rviz, but when you try to move the robot using the:

`rosrun ur_robot_driver test_move`

You’ll run into issues. If you are having issues with getting the external control to run, be sure that the external control has the Host IP setting as your own computer’s IP.

To find your own IP address, go to Setting > Network > Ethernet > Settings, and you should see a screen like this appear:

![img](https://lh7-us.googleusercontent.com/I8yY2LBpPD_v-mbBdi4drEnYsEeuVggwpi9tZ8yL7E2tB3F-c273y5XWuWN73gfdJPH2huF2cg_QxihuzTyFMREUNhW_MT7Hskqxtlri9grUDhwHpoz4KZvH2h2Zk89apLY0peJT-o2Wx9Mav15Z7h4)

Use the IPv4 Address in External Control, and then run the program.

**UR5e Setup on Ubuntu 20.04 and ROS Noetic Instructions:**

Requirements

Install UR Driver and FMauch’s fork of config files per https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/README.md

For Gazebo simulated ur5e:

In separate windows: 

`roslaunch ur_gazebo ur5e_bringup.launch`

`roslaunch ur5e_moveit_config ur5e_moveit_planning_execution.launch sim:=true`

`rosrun rviz rviz -f world`

Then in Rviz add MotionPlanning and change planning group to manipulator from endeffector

For simulated ur3e: as above but with 3 instead of 5

However, gazebo enforces collision with ground but this is not being forwarded to MoveIt’s PlanningScene.

To get moveit_commander working in Python, attempting a from source install of moveit’s master branch plus the fmauch ur descriptions. But catkin failed with a CPP compilation error in motion_planning_frame_manipulation.cpp.o

Determined that the deb packages had just not been installed. Ran sudo apt-get install ros-noetic-moveit and it added several packages including moveit_commander.

MoveIt struggles with path planning even for very easy asks with long time allotments. Works much better with joint limited version (add limited:=true to *_gazebo.launch and moveit_planning_execution.launch invocations), but it still isn’t perfect wrt avoiding collisions with the ground. In particular, seem to need to add the floor object twice before it is actually present in the planning scene. This may be a synchronization issue

For Real ur:

1. Release estop
2. In upper right, change from automatic or remote to manual mode. Password: biorobotics
3. In settings/system/URCaps make External Control Active (reboot if required)
4. In Installation/URCaps/External Control set the remote host IP to your machine
5. In Run, lhey don't go thoad surimoveitcontrol.upp OR create a new program with only an ExternalControl block
6. Power On
7. Start robot
8. Do: roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.0.102 kinematics_config:=/home/ggutow/NGtestbedrightcalibration.yaml
   1. Replace path to kinematics config file and robot_ip as necessary
9. Do: roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch limited:=true
10. Do: rosrun rviz rvis -f base
11. Press play on robot, terminal window should say “Robot connected to reverse interface. Ready to receive control commands.”

Get a VPN and have to ssh into the computer. Needs to learn UNIX first:

- SSH - connecting to a machine
- UNIX - get around file system, permissions, basic commands,
- Get around in the file system.
- Linux command line cheat sheet
- man command - manual pages
- Check what is running on a system
- basic utilities: listing all the processes that are running, what they are using.
- htop, etc.
- 

**Environment setup for using Python to control the UR5e**

1. **![img](https://lh7-us.googleusercontent.com/k18r4eIPue8urMRA3-g1i5U6ZtH0VUewQ0vG7pOTWtnFe6JW8VCb1lzHZZd4G40_bSrKCMlyv2ik2wu059t7Jdrsl9ECOg8qr3fsY76XJBAucSNQDJcAUX1kSdMn6KvzZHgrg8JJMAxVZAw99AtGWew)**

Add ros noetic path on your pycharm

1.  Install **rospy package** on your anaconda environment.(**pip install rospkg)**
2.  You can run the code.

(**PS**: Environment setup for UR5e, please reference this on [GitHub](https://github.com/gaolongsen/UR5e_ROS_Driver.git). I made some changes based on the original version to make sure the new version is suitable for our lab’s environment.)

**Smart Satellite Actuated Linear Rail**

When utilizing the Vernier Dual Range force sensor, the max force required to move the satellite is 32.78N, but the average force required is close to 18N. This high force point on the track is in the middle of the track, where two ends of aluminum extrusion connect. If the load of the satellite increases, it is likely that this max force would also increase.