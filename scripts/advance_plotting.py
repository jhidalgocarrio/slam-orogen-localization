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
	    
	plt.show(block=False)

    def plot_axis2(self, fign=1, axis=0, cov=False, levelconf=1, grid=False, linecolor=[1,0,0]):
	
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
	    plt.plot(self.t, sdmax, color=[0,0,0], linestyle='--')
	    plt.plot(self.t, sdmin, color=[0,0,0], linestyle='--')
	
	if False != grid:
	    plt.grid(True)
	    
	plt.show(block=False)

    def plot_errorbars(self, fign=1, axis=0, cov=False, levelconf=1, grid=False, linecolor=[1,0,0]):
	
	values = []
	error=[]
	for i in range(0,len(self.data)):
	    values.append(self.data[i][axis])
	    if False != cov:
	    	error.append((levelconf*sqrt(self.var[i][axis])))
            else:
                error.append(0)

	#print len(values)
	plt.figure(fign)
        plt.errorbar(self.t, values,
           yerr=error,
           marker='D',
           color='k',
           ecolor='r',
           lw=2,
           markerfacecolor=linecolor,
           capsize=0,
           linestyle='-')	
	
	if False != grid:
	    plt.grid(True)

	plt.show(block=False)

    def getAxis(self, axis=0):
	values = []
	for i in range(0,len(self.data)):
	    values.append(self.data[i][axis])

        return values

    def getStd(self, axis=0, levelconf=1):
	values = []
	for i in range(0,len(self.data)):
	    values.append((levelconf*sqrt(self.var[i][axis])))

        return values

    def getStdMax(self, axis=0, levelconf=1):
        values = []
	sdmax=[]
	for i in range(0,len(self.data)):
	    values.append(self.data[i][axis])
            sdmax.append(values[i]+(levelconf*sqrt(self.var[i][axis])))

        return sdmax

    def getStdMin(self, axis=0, levelconf=1):
        values = []
	sdmin=[]
	for i in range(0,len(self.data)):
	    values.append(self.data[i][axis])
            sdmin.append(values[i]-(levelconf*sqrt(self.var[i][axis])))

        return sdmin

class OneData:
    
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
	    self.data.append(np.array([float(row[1])]))
	    
	    if False != cov:
		matrix = np.array([[float(row[2])]])
		self.cov.append(matrix)
		self.var.append(sqrt(matrix))
		
	    
	time = self.time
	for i in range(0,len(time)-1):
	    tbody = float(time[i+1]) - float(time[i])
	    self.delta.append(tbody)
	    
	self.t = mean(self.delta) * r_[0:len(self.time)]
	
	    
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
	    
	plt.show(block=False)


#FrontEnd reference from Ground Truth
frontendreference100Hz = ThreeData()
#frontendreference100Hz.readData('data/normal_spacehall/frontend_referencepose_velocity.1154.0.data', cov=True)
frontendreference100Hz.readData('data/multitest_spacehall/frontend_referencepose_velocity.0940.1.data', cov=True)
frontendreference100Hz.eigenValues()

#FrontEnd Motion model velocity
frontendbody100Hz = ThreeData()
#frontendbody100Hz.readData('data/normal_spacehall/frontend_poseout_velocity.1154.1.data', cov=True)
frontendbody100Hz.readData('data/multitest_spacehall/frontend_poseout_velocity.0940.1.data', cov=True)
frontendbody100Hz.eigenValues()

#DeltaVelocity from MotionModel
backenddeltamodel10Hz = ThreeData()
backenddeltamodel10Hz.readData('data/normal_spacehall/backend_delta_velo_model.1154.1.data', cov=True)
backenddeltamodel10Hz.eigenValues()

#Acc from MotionModel
backendaccmodel10Hz = ThreeData()
backendaccmodel10Hz.readData('data/normal_spacehall/backend_acc_model.1154.1.data', cov=True)
backendaccmodel10Hz.eigenValues()

#DeltaVelocity from InertialState
backenddeltainertial10Hz = ThreeData()
backenddeltainertial10Hz.readData('data/normal_spacehall/backend_delta_velo_inertial.1154.1.data', cov=True)
backenddeltainertial10Hz.eigenValues()

#Acc from InertialState
backendaccinertial10Hz = ThreeData()
backendaccinertial10Hz.readData('data/normal_spacehall/backend_acc_inertial.1154.1.data', cov=True)
backendaccinertial10Hz.eigenValues()

#DeltaVelocity Error
backenddeltaerror10Hz = ThreeData()
backenddeltaerror10Hz.readData('data/normal_spacehall/backend_delta_velo_error.1154.0.data', cov=True)
backenddeltaerror10Hz.eigenValues()

#DeltaVelocity Common
backenddeltacommon10Hz = ThreeData()
backenddeltacommon10Hz.readData('data/normal_spacehall/backend_delta_velo_common.1154.0.data', cov=True)
backenddeltacommon10Hz.eigenValues()

#Hellinger Coeff
backendhellinger10Hz = ThreeData()
backendhellinger10Hz.readData('data/normal_spacehall/backend_hellinger.1154.0.data', cov=False)
backendhellinger10Hz.eigenValues()

#Bhattacharyya Coeff
backendbhatta10Hz = ThreeData()
backendbhatta10Hz.readData('data/normal_spacehall/backend_bhatta.1154.0.data', cov=False)
backendbhatta10Hz.eigenValues()

#Threshold Coeff
backendthreshold10Hz = ThreeData()
backendthreshold10Hz.readData('data/normal_spacehall/backend_threshold.1154.0.data', cov=False)
backendthreshold10Hz.eigenValues()

#Mahalanobis distance
backendmahalanobis10Hz = OneData()
backendmahalanobis10Hz.readData('data/normal_spacehall/backend_mahalanobis.1154.0.data', cov=False)

#Sensitivity analysis
sensitivitytstate = OneData()
sensitivitytstate.readData('data/normal_spacehall/frontend_sensitivity_tstate.1154.0.data', cov=False)

#Sensitivity analysis
sensitivitytcov = OneData()
sensitivitytcov.readData('data/normal_spacehall/frontend_sensitivity_tcovariance_RL.1154.0.data', cov=False)


frontendbody100Hz.plot_axis2(1, 0, True, 1, True, [0,1,1])
frontendbody100Hz.plot_errorbars(1, 0,True, 1, True, [0,1,1])
frontendbody100Hz.plot_axis(1, 1,False, 1, True, [0,1,0])
frontendbody100Hz.plot_axis(1, 2,False, 1, True, [1,0,0])

frontendreference100Hz.plot_axis(1, 0,False, 1, True, [0.5,1,0])
frontendreference100Hz.plot_axis(1, 1,False, 1, True, [1,0,0])

backenddeltamodel10Hz.plot_axis(1, 0, True, 1, True, [0,0,1])
backenddeltainertial10Hz.plot_axis(1, 0, True, 1, True, [0,1,0])
backenddeltacommon10Hz.plot_axis(1, 0, True, 1, True, [0.5,0.5,0.5])
backenddeltaerror10Hz.plot_axis(1, 0, True, 1, True, [1,0,0])
backendhellinger10Hz.plot_axis(1, 0, False, 1, True, [0,0,0])
backendbhatta10Hz.plot_axis(1, 0, False, 1, True, [0,0,0])
backendmahalanobis10Hz.plot_axis(1, 0, False, 1, True, [1,0,0])
backendthreshold10Hz.plot_axis(1, 0, False, 1, True, [0,0,0])

backenddeltamodel10Hz.plot_axis(2, 1, True, 1, True, [0,0,1])
backenddeltainertial10Hz.plot_axis(2, 1, True, 1, True, [0,1,0])
backenddeltaerror10Hz.plot_axis(2, 1, True, 1, True, [1,0,0])

backenddeltamodel10Hz.plot_axis(3, 2, True, 1, True, [0,0,1])
backenddeltainertial10Hz.plot_axis(3, 2, True, 1, True, [0,1,0])

backendaccmodel10Hz.plot_axis(2, 0, False, 1, True, [0,0,1])
backendaccinertial10Hz.plot_axis(2, 0, False, 1, True, [0,1,0])

velocommon = []
velocommon.append(backenddeltacommon10Hz.data[0][0])
for i in range(1,len(backenddeltacommon10Hz.t)):
    velocommon.append(velocommon[i-1] + backenddeltacommon10Hz.data[i][0])

plot(backenddeltacommon10Hz.t,velocommon, '-o', label="X common velocity", color='green')

veloinertial = []
veloinertial.append(backenddeltainertial10Hz.data[0][0])
for i in range(1,len(backenddeltainertial10Hz.t)):
    veloinertial.append(veloinertial[i-1] + backenddeltainertial10Hz.data[i][0])

plot(backenddeltainertial10Hz.t,veloinertial, '-o', label="X common velocity", color='red')


#Customized plotting
plt.figure(1)
errorbar(frontendbody100Hz.t, frontendbody100Hz.getAxis(0),
           yerr=frontendbody100Hz.getStd(0),
           marker='D',
           color='k',
           ecolor='r',
           alpha=0.5,
           markerfacecolor='b',
           label="",
           capsize=0,
           linestyle='--')
plt.grid(True)
plt.show(block=False)

plt.figure(1)
values = frontendbody100Hz.getAxis(0)
plt.plot(frontendbody100Hz.t, values,
        marker='.', label="Motion Model X-axis", color=[1,0,0], lw=2)
plt.plot(frontendbody100Hz.t, frontendbody100Hz.getStdMax(0, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 1\sigma$ uncertainty')
plt.plot(frontendbody100Hz.t, frontendbody100Hz.getStdMin(0, 3) , color=[0,0,0], linestyle='--', lw=2)
values=frontendreference100Hz.getAxis(0)
plt.plot(frontendreference100Hz.t, values,
        marker='D', label="Ground Truth X-axis", color=[0,0.5,0.5], alpha=0.5, lw=5)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/turn_point_x_velocity_plot.png')

plt.figure(1)
values = frontendbody100Hz.getAxis(1)
plt.plot(frontendbody100Hz.t, values,
        marker='.', label="Motion Model Y-axis", color=[0,1,0], lw=2)
plt.plot(frontendbody100Hz.t, frontendbody100Hz.getStdMax(1) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 1\sigma$ uncertainty')
plt.plot(frontendbody100Hz.t, frontendbody100Hz.getStdMin(1) , color=[0,0,0], linestyle='--', lw=2)
values=frontendreference100Hz.getAxis(1)
plt.plot(frontendreference100Hz.t, values,
        marker='D', label="Ground Truth Y-axis", color=[0,0.5,0.5], alpha=0.5, lw=5)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/turn_point_y_velocity_plot.png')

plt.figure(1)
values = frontendbody100Hz.getAxis(2)
plt.plot(frontendbody100Hz.t, values,
        marker='.', label="Motion Model Z-axis", color=[0,0,1], lw=2)
plt.plot(frontendbody100Hz.t, frontendbody100Hz.getStdMax(2) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 1\sigma$ uncertainty')
plt.plot(frontendbody100Hz.t, frontendbody100Hz.getStdMin(2) , color=[0,0,0], linestyle='--', lw=2)
values=frontendreference100Hz.getAxis(2)
plt.plot(frontendreference100Hz.t, values,
        marker='D', label="Ground Truth Z-axis", color=[0,0.5,0.5], alpha=0.5, lw=5)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/turn_point_z_velocity_plot.png')


