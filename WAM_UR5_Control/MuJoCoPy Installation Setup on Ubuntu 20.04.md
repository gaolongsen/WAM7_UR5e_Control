## **MuJoCoPy Installation Setup on Ubuntu 20.04**

**Step 1: Install anaconda**


 [Anaconda3-2021.04-Linux-x86_64.sh](https://repo.anaconda.com/archive/Anaconda3-2021.04-Linux-x86_64.sh)

`sudo chmod +x Anaconda3-2021.04-Linux-x86_64.sh`

`./Anaconda3-2021.04-Linux-x86_64.sh`


 **Step 2 : install git**

`sudo apt install git`


 **Step 3 : install the** **mujoco** **library**



1. Download the Mujoco library from 

https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz

2. create a hidden folder :

 `sudo mkdir /home/username/.mujoco`

(Note: if the folder you created without any permission, use “**sudo** **chmod** **a+rwx**   **/path/to/file**” to give the folder permission)



3. extract the library to the .mujoco folder
4. include these lines in .bashrc file(terminal command: **gedit** **~/.****bashrc**):

export LD_LIBRARY_PATH=/home/user_name/.mujoco/mujoco210/bin

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia

export PATH="$LD_LIBRARY_PATH:$PATH"

export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so



5. `source ~/.bashrc`



6. Test that the library is installed by going into:

 `cd ~/.mujoco/mujoco210/bin`

`./simulate ../model/humanoid.xml`



 **Step 4 Install** **mujoco-py****:**

`conda create --name mujoco_py python=3.8`

`conda activate mujoco_py`

`sudo apt update`

`sudo apt-get install patchelf`

`sudo apt-get install python3-dev build-essential libssl-dev libffi-dev libxml2-dev` 

`sudo apt-get install libxslt1-dev zlib1g-dev libglew1.5 libglew-dev python3-pip`

`git clone https://github.com/openai/mujoco-py`

`cd mujoco-py`

`pip install -r requirements.txt`

`pip install -r requirements.dev.txt`


 `pip3 install -e . --no-cache`


 **Step 5** **reboot your machine**



 **Step 6** **run these commands**

`conda activate mujoco_py`

`sudo apt install libosmescd examplesa6-dev libgl1-mesa-glx libglfw3`

`sudo apt-get install libosmesa6-dev`

`sudo ln -s /usr/lib/x86_64-linux-gnu/libGL.so.1` 

`sudo ln -s /usr/lib/x86_64-linux-gnu/libGL.so`

`pip3 install -U 'mujoco-py<2.2,>=2.1'`

`cd examples`

`python3 setting_state.py`



 **If you’re getting a** **Cython** **error, try:**

`pip install "cython<3"`