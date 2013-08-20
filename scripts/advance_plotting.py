#!/usr/bin/env python

import csv, scipy
from pylab import *
import numpy as np
import matplotlib.pyplot as plt

def func(x):
    return (x-3)*(x-5)*(x-7)+85
    
class ThreeData:
    
    def __init__(self):
	self.time = []
	self.delta = []
	self.t=[]
	self.data=[]
	self.cov=[]
	self.var=[]
	
    def f(self):
        return 'hello world'
    
    def readData(self,filename, cov=False):
	
	for row in csv.reader(open(filename, 'rb'), delimiter=' ', quotechar='|'):
	    #print row
	    self.time.append(float(row[0])/1000000)
	    self.data.append(np.array([float(row[1]), float(row[2]), float(row[3])]))
	    
	    if False != cov:
		matrix = np.array([[float(row[4]), float(row[5]), float(row[6])],
			    [float(row[7]), float(row[8]), float(row[9])],
			    [float(row[10]), float(row[11]), float(row[12])]])
		self.cov.append(matrix)
		
	    
	time = self.time
	for i in range(0,len(time)-1):
	    tbody = float(time[i+1]) - float(time[i])
	    self.delta.append(tbody)
	    
	self.t = mean(self.delta) * r_[0:len(self.time)]
	
    def eigenValues(self):
	
	#Eigen values are the axis of the ellipsoid
	for i in range(0,len(self.cov)):
	    self.var.append(linalg.eigvals(self.cov[i]))
	    
    def plot_axis(self, fign=1, axis=0, cov=False, levelconf=1, grid=False, linecolor=[1,0,0]):
	
	values = []
	sdmax=[]
	sdmin=[]
	for i in range(0,len(self.data)):
	    values.append(self.data[i][axis])
	    if False != cov:
	    	sdmax.append(values[i]+(levelconf*sqrt(self.var[i][axis])))
	    	sdmin.append(values[i]-(levelconf*sqrt(self.var[i][axis])))
	    #print i
	    #print values[i]
	    
	#print len(values)
	plt.figure(fign)
	plt.plot(self.t, values, '-o', label="X axis", color=linecolor)
	
	if False != cov:
	    plt.fill_between(self.t, sdmax, sdmin, color=linecolor)
	
	if False != grid:
	    plt.grid(True)
	    
	plt.show()
	

#Motion model incremental velocity
ivelbody100Hz = ThreeData()
ivelbody1Hz = ThreeData()
ivelbody100Hz.readData('data/normal_spacehall/spacehall1140.puremodel_velo.incre.slip_gp.5.data', cov=True)
ivelbody1Hz.readData('data/normal_spacehall/spacehall1140.puremodel_velo.incre.slip_gp.6.data', cov=True)
ivelbody100Hz.eigenValues()
ivelbody1Hz.eigenValues()

#IMU incremental velocity
velimu100Hz = ThreeData()
velimu1Hz = ThreeData()
velimu100Hz.readData('data/normal_spacehall/spacehall1140.imu_acc_velo.incre.slip_gp.5.data', cov=True)
velimu1Hz.readData('data/normal_spacehall/spacehall1140.imu_acc_velo.incre.slip_gp.6.data', cov=True)
velimu100Hz.eigenValues()
velimu1Hz.eigenValues()


#Velocity error
evelocity100Hz = ThreeData()
evelocity1Hz = ThreeData()
evelocity100Hz.readData('data/normal_spacehall/spacehall1140.velocity_error.slip_gp.5.data', cov=True)
evelocity1Hz.readData('data/normal_spacehall/spacehall1140.velocity_error.slip_gp.6.data', cov=True)
evelocity100Hz.eigenValues()
evelocity1Hz.eigenValues()


#Ploting
ivelbody100Hz.plot_axis(1, 0,True, 1, True, [0,0,1])
velimu100Hz.plot_axis(1, 0,True, 1, True, [0,1,0])
evelocity100Hz.plot_axis(1, 0,True, 1, True, [1,0,0])

ivelbody1Hz.plot_axis(3, 0,True, 2, True, [0,0,1])
velimu1Hz.plot_axis(3, 0,True, 2, True, [0,1,0])
#evelocity1Hz.plot_axis(2, 0,True, 1, True, [1,0,0])



#Motion model incremental velocity
ivelbody = ThreeData()
ivelbody.readData('data/normal_spacehall/spacehall1140.puremodel_velo.incre.0.data', cov=True)
ivelbody.eigenValues()

#IMU incremental velocity
velimu = ThreeData()
velimu.readData('data/normal_spacehall/spacehall1140.imu_acc_velo.incre.0.data', cov=True)
velimu.eigenValues()


#Velocity error
evelocity = ThreeData()
evelocity.readData('data/normal_spacehall/spacehall1140.velocity_error.0.data', cov=True)
evelocity.eigenValues()

ivelbody.plot_axis(3, 0,True, 3, True, [0,0,1])
velimu.plot_axis(3, 0,True, 3, True, [0,1,0])
evelocity.plot_axis(3,0,True, 1, True, [1,0,0])


#Motion model incremental velocity
ivelbody = ThreeData()
ivelbody.readData('data/normal_spacehall/spacehall1140.puremodel_velo.incre.1.data', cov=True)
ivelbody.eigenValues()

#IMU incremental velocity
velimu = ThreeData()
velimu.readData('data/normal_spacehall/spacehall1140.imu_acc_velo.incre.1.data', cov=True)
velimu.eigenValues()


#Velocity error
evelocity = ThreeData()
evelocity.readData('data/normal_spacehall/spacehall1140.velocity_error.1.data', cov=True)
evelocity.eigenValues()

ivelbody.plot_axis(7, 0,True, 3, True, [0,0,1])
velimu.plot_axis(7, 0,True, 3, True, [0,1,0])
evelocity.plot_axis(6,1,True, 1, True, [1,0,0])


#FrontEnd Motion model velocity
frontendbody100Hz = ThreeData()
frontendbody100Hz.readData('data/normal_spacehall/frontend_poseout_velocity.1154.3.data', cov=True)
frontendbody100Hz.eigenValues()

frontendreference100Hz = ThreeData()
frontendreference100Hz.readData('data/normal_spacehall/frontend_referencepose_velocity.1154.3.data', cov=True)
frontendreference100Hz.eigenValues()

frontendbody100Hz.plot_axis(1, 0,False, 1, True, [0,0,1])
frontendbody100Hz.plot_axis(1, 1,False, 1, True, [0,1,0])
frontendbody100Hz.plot_axis(1, 2,False, 1, True, [1,0,0])


frontendreference100Hz.plot_axis(1, 0,False, 1, True, [0,0,1])
frontendreference100Hz.plot_axis(1, 1,False, 1, True, [1,0,0])


