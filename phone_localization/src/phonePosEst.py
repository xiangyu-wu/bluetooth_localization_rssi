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
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

vehiclePosTopicType = rospy.get_param('vehicle_pos_topic_type')
vehiclePosTopicName = rospy.get_param('vehicle_pos_topic_name')

currentVehiclePosition = np.array([0.0, 0.0, 0.0], dtype=np.float32)

######################################################################
######################################################################
bluetoothModuleAddr = "5C:F3:70:8A:38:B6" #the addrees of the bluetooth module to be used
loopFrequency = 50 #[Hz] the frequency of the loop
noDeviceTimeout = 1.0 #[s] time threshold for foggetting a bluetooth device
durationOfDiscoveryStep = 5  #[s] time for searching BT devices
######################################################################
######################################################################

currentBTdeviceList = []
allBTdeviceList = []
major_classes = ( "Miscellaneous", 
                  "Computer", 
                  "Phone", 
                  "LAN/Network Access point", 
                  "Audio/Video", 
                  "Peripheral", 
                  "Imaging" )

devID = bluetooth._bluetooth.hci_devid(bluetoothModuleAddr)


class phonePositionEstimate:
    def __init__(self, BTname, BTaddress, BTclass, btModuleAddr):
        #info about estimated phone position
        self.cumPos = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.maxRSSI = -np.inf
        self.statusRSSI = 0
        self.currentRSSI = -np.inf
        self.dataNum = 0
        self.lastRSSITime = rospy.get_rostime()
        
        #some properties about the bluetooth device
        self.BTname = BTname
        self.BTaddress = BTaddress
        self.BTclass = BTclass
        
        #the object for getting RSSI value of a bluetooth device
        self.btrssi = BluetoothRSSI(BTaddress, btModuleAddr)
        
    def updatePhonePosition(self):
        #get the current RSSI of BT device
        infoRSSI = self.btrssi.get_rssi()
        
        #For error code definition, please refer to bluetooth core specifications
        if infoRSSI == None:
            #phone not visible
            self.statusRSSI = 8 #bluetooth connection failure error code
            self.currentRSSI = -np.inf #if fail, make current RSSI -inf
            return False
        elif infoRSSI[0] != 0:
            #having error getting RSSI
            self.statusRSSI = infoRSSI[0]
            self.currentRSSI = -np.inf
            return False
        else:
            self.statusRSSI = infoRSSI[0]
            self.currentRSSI = infoRSSI[1]
        
        if self.currentRSSI == self.maxRSSI:
            self.cumPos = self.cumPos + currentVehiclePosition
            self.dataNum += 1
        
        elif self.currentRSSI > self.maxRSSI:
            self.maxRSSI = self.currentRSSI
            self.cumPos = currentVehiclePosition
            self.dataNum = 1
            
        return True

        
def deviceDiscovery(lock):
    while not rospy.is_shutdown():
        #Search nearby bluetooth devices
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
                
            #check if device found already in the current device list
            inCurrentList = False
            lock.acquire()
            for d in currentBTdeviceList:
                if d.BTaddress == addr:
                    inCurrentList = True
                    break
            lock.release()
            
            if not inCurrentList:
                #check whether that device was found before:
                #if was found before, just add the corresponding pos estimation object
                #back to the current list
                foundPreviously = False
                
                for d in allBTdeviceList:
                    if d.BTaddress == addr:
                        d.btrssi = BluetoothRSSI(addr, bluetoothModuleAddr) #restart getting RSSI
                        d.lastRSSITime = rospy.get_rostime() 
                        d.updatePhonePosition() #reconnect to the phone and get its RSSI
                        lock.acquire()
                        currentBTdeviceList.append(d)#add device to the current list
                        lock.release()
                        foundPreviously = True
                        break
        
                #if not found before, add to both the current and all time BT device lists
                if not foundPreviously:
                    newDevice = phonePositionEstimate(name, addr, category, bluetoothModuleAddr)
                    newDevice.updatePhonePosition() #connect to the phone and get its RSSI
                    lock.acquire()
                    currentBTdeviceList.append(newDevice) #add device to the current list
                    lock.release()
                    allBTdeviceList.append(currentBTdeviceList[-1])
          
                         
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
       
def publisherPhonePosMarker(posx, posy, posz):
    estPhonePosMarker.header.stamp = rospy.get_rostime()
    estPhonePosMarker.header.frame_id = "/my_frame"
    estPhonePosMarker.ns = 'bluetooth_phone_localization'
    estPhonePosMarker.action = estPhonePosMarker.ADD
    estPhonePosMarker.pose.orientation.w = 1.0
    estPhonePosMarker.id = 0
    estPhonePosMarker.type = estPhonePosMarker.POINTS
    estPhonePosMarker.scale.x = 0.1
    estPhonePosMarker.scale.y = 0.1
    estPhonePosMarker.color.g = 1.0
    estPhonePosMarker.color.a = 1.0
    
    markerPos.x = estPhonePosMsg.est_posx
    markerPos.y = estPhonePosMsg.est_posy
    markerPos.z = estPhonePosMsg.est_posz
    estPhonePosMarker.points.append(markerPos)
    estPosMarkerPublisher.publish(estPhonePosMarker)
    
rospy.init_node('phonePosEst', anonymous=True)

if vehiclePosTopicType == 'nav_msgs/Odometry':
    rospy.Subscriber(vehiclePosTopicName, Odometry, callbackVehiclePos)
elif vehiclePosTopicType == 'geometry_msgs/PoseStamped':
    rospy.Subscriber(vehiclePosTopicName, PoseStamped, callbackVehiclePos)
else:
    raise AssertionError('Vehicle postion message type not correct!')
    
estPosPublisher = rospy.Publisher("phone_position", phone_pos_est, queue_size=10)
estPhonePosMsg = phone_pos_est()

estPosMarkerPublisher = rospy.Publisher("phone_marker", Marker, queue_size = 10)
estPhonePosMarker = Marker()
markerPos = Point() 

loopRate = rospy.Rate(loopFrequency)

threadRosSpin = threading.Thread(name='rosSpin', target=rosSpin)
threadRosSpin.start()

lock = threading.Lock() #use lock to prevent changing the currentBTdeviceList at the same time
threadDeviceDiscovery = threading.Thread(name='discovery', target=deviceDiscovery, args=(lock,))
threadDeviceDiscovery.start()

while not rospy.is_shutdown():
    
    if len(currentBTdeviceList) != 0:
        #get the RSSI value of devices on the list
        
        lock.acquire()
        for device in currentBTdeviceList:
            #if successfully got the RSSI, update last successful time
            if device.updatePhonePosition():
                device.lastRSSITime = rospy.get_rostime()
                
            estPhonePosMsg.header.stamp = rospy.get_rostime()
            estPhonePosMsg.name = device.BTname
            estPhonePosMsg.address = device.BTaddress
            estPhonePosMsg.category = device.BTclass
            if not device.dataNum == 0:
                estPhonePosMsg.est_posx = device.cumPos[0]/device.dataNum
                estPhonePosMsg.est_posy = device.cumPos[1]/device.dataNum
                estPhonePosMsg.est_posz = device.cumPos[2]/device.dataNum
            else:
                estPhonePosMsg.est_posx = device.cumPos[0]
                estPhonePosMsg.est_posy = device.cumPos[1]
                estPhonePosMsg.est_posz = device.cumPos[2]
            estPhonePosMsg.rssi_status = device.statusRSSI
            estPhonePosMsg.rssi = device.currentRSSI
            estPhonePosMsg.maxrssi =  device.maxRSSI

            estPosPublisher.publish(estPhonePosMsg)
            
            publisherPhonePosMarker(estPhonePosMsg.est_posx, estPhonePosMsg.est_posy, estPhonePosMsg.est_posz)
            
            #if haven't heard from the devices for > noDeviceTimeout, remove from current list
            if (rospy.get_rostime()-device.lastRSSITime) > rospy.Duration(noDeviceTimeout):
                currentBTdeviceList.remove(device)  
        lock.release()
            
    #keep the RSSI inquiry frequency at the desired value
    loopRate.sleep()

threadDeviceDiscovery.join()
