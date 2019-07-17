# bluetooth_localization
This Repo contains a ROS package -- **phone_localization**, which uses bluetooth RSSI to localize phones. There is only one ROS node in the package -- **phonePosEst**. The ROS node searchs for nearby bluetooth devices and gets their RSSI value, at a custom specified frequency (maximum posible rate is about 800Hz, the default rate is 50Hz). It subscribes to the position message of the vehicle and publishes the name, category (phone, laptop, ect.), address, RSSI and estimated position of each detected phones. The position of the phone is estimated to be the centroid of the vehicle's positions with the largest RSSI value.

# 1-download this Repo
`cd` to a directory convenient to you and download this Repo by `git clone https://github.com/muellerlab/bluetooth_localization.git`

# 2-install libbluetooth-dev module
For compilation based on BlueZ(the official Linux Bluetooth protocol stack), you need to install libbluetooth-dev module for compiling your code by: `sudo apt-get install libbluetooth-dev`

# 3-install the PyBluez module
The PyBluez module allows Python code to access the host machine's Bluetooth resources: `pip install pybluez`

# 4-install the bluetooth-proximity module
The bluetooth-proximity module is used for getting the RSSI value of a Bluetooth device by address. Do a small modification and install it by:
* `git clone https://github.com/ewenchou/bluetooth-proximity.git`
*  replace the file /bt_proximity/bt_rssi.py with the bt_rssi.py file in the bluetooth_localization Repo (the repo with this README)
* `cd bluetooth-proximity`
* `sudo python setup.py install`

# 5-setup the workspace 
Create a ROS workspace:
`mkdir -p ~/bluetooth_ws/src`

`cd` into the the bluetooth_ws folder and link the phone_localization package by:
* `cd ~/bluetooth_ws/`
* `ln -s ~/YOUR BLUETOOTH LOCALIZATION REPO DIRECTORY/phone_localization src/`

Make the python script executable by:
`chmod +x ~/bluetooth_ws/src/phone_localization/src/phonePosEst.py`


# 6-set parameters and compile the codes
Plug the usb bluetooth dongle to the NUC. Get the address of the installed bluetooth modules using `bluetoothctl`. The address is a unique identifier of a bluetooth module. Change the bluetoothModuleAddr in **phonePosEst.py** to the address of the bluetooth module you want to use. If you are unable to distinguish which address correponds to the bluetooth dongle, just unplug it, run `bluetoothctl` again and see which device disappears.

In addition, in **phone_localization.launch**, please change the parameter named **vehicle_pos_topic_type** to the type of the position estimation message of the vehicle, and also change the **vehicle_pos_topic_name** to the name of that topic.  

Compile the code:
* `cd ~/bluetooth_ws/`
* `catkin_make`

# 7-run the codes 
Turn on the bluetooth of both the NUC and the phones whose position needs to be estimated. Run the ROS node by:
* `cd ~/bluetooth_ws`
* `source /devel/setup.bash`
* `roslaunch phone_localization phone_localization.launch`

# 8-analyze the recorded ROS bag file
First the recorded bag file to a pickle file by running
* `python convertToPickle.py NAME_OF_THE_BAG_FILE`
