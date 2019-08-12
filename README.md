# bluetooth_localization
Please contact Xiangyu Wu if you have trouble with this Repo. Phone: (510)3651730  Email: wuxiangyu@berkeley.edu

This Repo contains a ROS package -- **phone_localization**, which uses bluetooth RSSI to localize phones. There is only one ROS node in the package -- **phonePosEst**. The ROS node searchs for nearby bluetooth devices and gets their RSSI value, at a custom specified frequency (maximum posible rate is about 800Hz, the default rate is 50Hz). It subscribes to the position message of the vehicle and publishes the name, category (phone, laptop, ect.), address, RSSI and estimated position of each detected phones. The position of the phone is estimated to be the centroid of the vehicle's positions with the largest RSSI value. Experiments show that if the vehicle's cloest distance to the phone is about 1.8m, the position estimation error of this package is about 2m.

The package does device discovery and getting the RSSI value of the device at the same time, only bluetooth device in the "Phone" category is considered. It should be used together with the usb bluetooth dongle, because the bluetooth module on the NUC on the vehicle is often bad for discovering nearby bluetooth devices, because of the block of bluetooth signal by the vehicle body.

It publishes three topics: /phone_position, /bluetooth_phone_report and /phone_marker. 
* The /phone_position publishes information about discovered bluetooth devices: address, name, category, estimated position, rssi value and maximum rssi value. 
* The /bluetooth_phone_report topic has the same topic type as /phone_position (a custom type: phone_pos_est), it reports the estimated postion of a phone if: 
  * (1) the maximum rssi value of that device is above a threshold, which prevents reporting a phone that is always far away from the vehicle during flight and thus has large position estimation error. 
  * (2) the phone is not connected for 20 seconds, which makes sure that the vehicle is indeed already far away from the phone and all the possible data is gathered. the value is set to 20 because sometimes bluetooth connection to a phone is unstable and got lost for about 10 seconds.
  * (3) the phone is not reported before, which prevents repeated artifact report
* The /phone_marker publishes the real-time position of all detected phones as spheres (radius=0.2m, green color), which can be visualized in rviz.

# 1-download this Repo
`cd` to a directory convenient to you and download this Repo by `git clone https://github.com/muellerlab/bluetooth_localization.git`

# 2-install libbluetooth-dev module
For compilation based on BlueZ(the official Linux Bluetooth protocol stack), you need to install libbluetooth-dev module for compiling your code by: `sudo apt-get install libbluetooth-dev`

# 3-install the PyBluez module
The PyBluez module allows Python code to access the host machine's Bluetooth resources. Install it by: `pip install pybluez`

# 4-install the bluetooth-proximity module
The bluetooth-proximity module is used for getting the RSSI value of a Bluetooth device by address. Do a small modification and install it by:
* `git clone https://github.com/ewenchou/bluetooth-proximity.git`
*  replace the file /bt_proximity/bt_rssi.py with the bt_rssi.py file in the bluetooth_localization Repo (the Repo of this README)
* `cd bluetooth-proximity`
* `sudo python setup.py install`

# 5-setup the workspace 
Create a ROS workspace:
`mkdir -p ~/bluetooth_ws/src`

`cd` into the the bluetooth_ws folder and link the phone_localization package by:
* `cd ~/bluetooth_ws/`
* `ln -s ~/YOUR_BLUETOOTH_LOCALIZATION_REPO_DIRECTORY/phone_localization src/`

Make the python script executable by:
`chmod +x ~/bluetooth_ws/src/phone_localization/src/phonePosEst.py`


# 6-set parameters and compile the codes
Power off the bluetooth module on the NUC to make sure the package uses the usb bluetooth dongle. 
Before you plug in the bluetooth dongle, get the address of the NUC's bluetooth modules using `bluetoothctl`. The address (e.g. 94:65:2D:22:5F:4C) is a unique identifier of a bluetooth module, and it won't change.
In phonePosEst.py, change the bluetooth module address in the command **"echo 'select 94:65:2D:22:5F:4C\npower off\nquit' | bluetoothctl"** to the bluetooth module of your NUC to power if off. Don't change the format of this command(e.g. don't add or delete space in it).

Plug the [Plugable bluetooth dongle](https://www.amazon.com/Plugable-Bluetooth-Adapter-Raspberry-Compatible/dp/B009ZIILLI/) to the NUC. You don't need to specify the address of the usb bluetooth dongle.

In addition, in phone_localization.launch, please change the parameter named **vehicle_pos_topic_type** to the type of the position estimation message of the vehicle, and also change the **vehicle_pos_topic_name** to the name of that topic.

Compile the code:
* `cd ~/bluetooth_ws/`
* `catkin_make`

# 7-run the codes 
Turn on the bluetooth of both the NUC and the phones whose position need to be estimated. Run the ROS node by:
* `cd ~/bluetooth_ws`
* `source devel/setup.bash`
* `roslaunch phone_localization phone_localization.launch`

# 8-analyze the recorded ROS bag file
First the recorded bag file to a pickle file by running `python convertToPickle.py NAME_OF_THE_BAG_FILE`

After that, run plot.py to plot the RSSI versus time, estimated position versus time. It also prints out the discovered bluetooth devices' address, name and category. You need to modify the file name and directory vairables according to the .pickle file you want to analyze. 
