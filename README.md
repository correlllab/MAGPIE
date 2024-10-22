# DeliGrasp



# Dependencies
* Python 3.10
* `ur_rtde`
    1. `sudo add-apt-repository ppa:sdurobotics/ur-rtde` 
    1. `sudo apt-get update`
    1. `sudo apt install librtde librtde-dev`
    1. `python3.9 -m pip install ur_rtde --user`
* Python Imaging Library (Fork)
    1. Remove conflicting installations
    1. `python3.9 -m pip install Pillow --user`
* Python Libraries: 
    1. `python3.9 -m pip install spatialmath-python splines py-trees pybullet --user`
* Servo Communication
    1. `python3.9 -m pip install dynamixel-sdk pyax12 --user`
    1. `sudo cp openCM.rules /etc/udev/rules.d/`
    1. `sudo adduser $USER dialout`
    1. Logout or Restart