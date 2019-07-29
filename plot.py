import numpy as np
import matplotlib.pyplot as plt
from convertToPickle import get_data

######################################################################
######################################################################
fileDir = 'data/phoneLocalization/' #directory of the log file 
fileName = 'test1' #the name of the log file (in .pickle format)
#address of a detected bluetooth device whose data you want to analyze
deviceAddr = '24:69:A5:92:36:C3' 
######################################################################
######################################################################

dat, fname = get_data([' ', fileDir+fileName+'.pickle'])

'''
Display info of all detected devices
'''
print('All the bluetooth devices found:')
btEstData = dat['/phone_position']

detectedDevices = np.unique(btEstData['address'])
for i in range(len(detectedDevices)):
    index = np.where(btEstData['address']==detectedDevices[i])[0][0]
    print("Device found "+str(i+1)+":")
    print('name: '+ btEstData['name'][index])
    print('category: '+ btEstData['category'][index])
    print('address: '+ btEstData['address'][index]+'\n')

'''
Plot RSSI and position estimation of a specific bluetooth device.
Specified by "deviceAddr".
'''
plot = plt.figure()
plot.add_subplot(3, 1, 1)
plot.add_subplot(3, 1, 2, sharex=plot.axes[0])
plot.add_subplot(3, 1, 3, sharex=plot.axes[0])

index = btEstData['address']==deviceAddr
 
plot.axes[0].plot(btEstData['t'][index], btEstData['rssi'][index], 'b.-')
plot.axes[0].set_ylabel('rssi value')

plot.axes[1].plot(btEstData['t'][index], btEstData['est_posx'][index], 'r.', label='x')
plot.axes[1].plot(btEstData['t'][index], btEstData['est_posy'][index], 'g.', label='y')
plot.axes[1].plot(btEstData['t'][index], btEstData['est_posz'][index], 'b.', label='z')
plot.axes[1].set_ylabel('position est (m)')
plot.axes[1].legend()

plot.axes[2].plot(btEstData['t'][index], btEstData['rssi_status'][index], 'b.')
plot.axes[2].set_ylabel('rssi status (debug value)')
plot.axes[2].set_xlabel('time (s)')

finalEstPos = np.asarray([btEstData['est_posx'][index][-1], 
                          btEstData['est_posy'][index][-1], btEstData['est_posz'][index][-1]])
print('Final estimated position of the *specified* bluetooth device:')
print("est_posx: %.3f m" % finalEstPos[0])
print("est_posy: %.3f m" % finalEstPos[1])
print("est_posz: %.3f m" % finalEstPos[2])

print('RSSI frequency: %0.3f' % (len(btEstData['t'][index])/(btEstData['t'][-1]-btEstData['t'][0])))

plt.show()
