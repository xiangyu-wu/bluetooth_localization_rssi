#!/usr/bin/env python

import numpy as np
import rospy
import threading
import bluetooth
import select
from bt_proximity import BluetoothRSSI
from phone_localization.msg import phone_pos_est
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

vehiclePosTopicType = rospy.get_param('vehicle_pos_topic_type')
vehiclePosTopicName = rospy.get_param('vehicle_pos_topic_name')

currentVehiclePosition = np.array([0.0, 0.0, 0.0], dtype=np.float32)

######################################################################
######################################################################
bluetoothModuleAddr = "5C:F3:70:8A:38:B6" #the addrees of the bluetooth module to be used
loopFrequency = 50 #[Hz] the frequency of the loop
noDeviceTimeout = 2.0 #[s] time threshold for foggetting a bluetooth device
durationOfDiscoveryStep = 5  #[s] time for searching BT devices
######################################################################
######################################################################


class phonePositionEstimate:
    def __init__(self, BTname, BTaddress, BTclass, btModuleAddr):
        #info about estimated phone position
        self.cumPos = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.maxRSSI = -np.inf 
        self.currentRSSI = -np.inf
        self.dataNum = 0
        #some properties about the bluetooth device
        self.BTname = BTname
        self.BTaddress = BTaddress
        self.BTclass = BTclass
        
        #the object for getting RSSI value of a bluetooth device
        self.btrssi = BluetoothRSSI(BTaddress, btModuleAddr)
        
    def updatePhonePosition(self):
        #get the current RSSI of BT device
        self.currentRSSI = self.btrssi.get_rssi()
        
        if self.currentRSSI == None:
            #phone not visible, nothing to do.
            return False
        
        if self.currentRSSI == self.maxRSSI:
            self.cumPos = self.cumPos + currentVehiclePosition
            self.dataNum += 1
        
        elif self.currentRSSI > self.maxRSSI:
            self.maxRSSI = self.currentRSSI
            self.cumPos = currentVehiclePosition
            self.dataNum = 1
            
        return True

def rosSpin():
    rospy.spin()

def callbackVehiclePos(vehiclePos):
    if vehiclePosTopicType == 'nav_msgs/Odometry':
        currentVehiclePosition[0] = vehiclePos.pose.pose.position.x
        currentVehiclePosition[1] = vehiclePos.pose.pose.position.y
        currentVehiclePosition[2] = vehiclePos.pose.pose.position.z
    elif vehiclePosTopicType == 'geometry_msgs/PoseStamped':
        currentVehiclePosition[0] = vehiclePos.pose.position.x
        currentVehiclePosition[1] = vehiclePos.pose.position.y
        currentVehiclePosition[2] = vehiclePos.pose.position.z

rospy.init_node('phonePosEst', anonymous=True)

if vehiclePosTopicType == 'nav_msgs/Odometry':
    rospy.Subscriber(vehiclePosTopicName, Odometry, callbackVehiclePos)
elif vehiclePosTopicType == 'geometry_msgs/PoseStamped':
    rospy.Subscriber(vehiclePosTopicName, PoseStamped, callbackVehiclePos)
else:
    raise AssertionError('Vehicle postion message type not correct!')
    
estPosPublisher = rospy.Publisher("phone_position", phone_pos_est, queue_size=10)
threadRosSpin = threading.Thread(target=rosSpin)
threadRosSpin.start()
estPhonePosMsg = phone_pos_est()

loopRate = rospy.Rate(loopFrequency)

currentBTdeviceList = []
allBTdeviceList = []
lastRSSITime = rospy.get_rostime() #stores the last time of getting RSSI value in float seconds.

major_classes = ( "Miscellaneous", 
                  "Computer", 
                  "Phone", 
                  "LAN/Network Access point", 
                  "Audio/Video", 
                  "Peripheral", 
                  "Imaging" )

devID = bluetooth._bluetooth.hci_devid(bluetoothModuleAddr)

while not rospy.is_shutdown():
    #Search when there's currently no bluetooth devices
    if len(currentBTdeviceList) == 0:
        nearby_devices = bluetooth.discover_devices(
            duration=durationOfDiscoveryStep, lookup_names=True, flush_cache=True, 
            lookup_class=True, device_id = devID)
        
        #add devices found to the current device list
        for addr, name, device_class in nearby_devices:
            major_class = (device_class >> 8) & 0xf
            if major_class < 7:
                category = major_classes[major_class]
            else:
                category = "Uncategorized"            
            
            #check whether that device was found before:
            #if was found before, just add the corresponding pos estimation object
            #back to the current list
            foundPreviously = False
            #for i in range(len(allBTdeviceList)):
            for d in allBTdeviceList:
                if d.BTaddress == addr:
                    currentBTdeviceList.append(d)
                    foundPreviously = True
                    break


            #if not found before, add to both the current and all time BT device lists
            if not foundPreviously:
                currentBTdeviceList.append(
                    phonePositionEstimate(name, addr, category, bluetoothModuleAddr))  
                allBTdeviceList.append(currentBTdeviceList[-1])

    else:
        #have BT devices:

        #get the RSSI value of devices on the list
        for device in currentBTdeviceList:
            #if the function returns true, it publishes the updated position
            if device.updatePhonePosition():
                lastRSSITime = rospy.get_rostime()
                
                estPhonePosMsg.header.stamp = rospy.get_rostime()
                estPhonePosMsg.name = device.BTname
                estPhonePosMsg.address = device.BTaddress
                estPhonePosMsg.category = device.BTclass
                estPhonePosMsg.est_posx = device.cumPos[0]/device.dataNum
                estPhonePosMsg.est_posy = device.cumPos[1]/device.dataNum
                estPhonePosMsg.est_posz = device.cumPos[2]/device.dataNum
                estPhonePosMsg.rssi = device.currentRSSI
                estPhonePosMsg.maxrssi =  device.maxRSSI
                estPosPublisher.publish(estPhonePosMsg)
            

        if (rospy.get_rostime()-lastRSSITime) > rospy.Duration(noDeviceTimeout):
            #haven't heard from the devices...
            currentBTdeviceList = [] 

            
    #keep the RSSI inquiry frequency at the desired value
    loopRate.sleep()
