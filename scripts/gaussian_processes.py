#!/usr/bin/env python

import csv, scipy, infpy, numpy, pylab
from pylab import *
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

import infpy.gp.kernel_short_names as kernels

# a function to save plots
def save_fig(prefix):
    "Save current figure in extended postscript and PNG formats."
    pylab.savefig('%s.png' % prefix, format='PNG')

#Load model velocity
spamReader = csv.reader(open('data/normal_spacehall/spacehall1140.puremodel_velo.slip_gp.5.data', 'rb'), delimiter=' ', quotechar='|')
#spamReader = csv.reader(open('data/normal_spacehall/spacehall1154.puremodel_velo.nk.data', 'rb'), delimiter=' ', quotechar='|')

timebody=[]
velbodyx=[]
velbodyy=[]
velbodyz=[]

for row in spamReader:
    #print row
    timebody.append(float(row[0])/1000000)
    velbodyx.append(float(row[1]))
    velbodyy.append(float(row[2]))
    velbodyz.append(float(row[3]))
    
    
deltabody = []
for i in range(0,len(timebody)-1):
    #print timebody[i]
    tbody = float(timebody[i+1]) - float(timebody[i])
    deltabody.append(tbody)
    
deltabody_t = mean(deltabody)    
sample_ratebody = 1/deltabody_t
tbody = deltabody_t * r_[0:len(timebody)]

#Vicon
spamReader = csv.reader(open('data/normal_spacehall/spacehall1140.vicon_velo.slip_gp.5.data', 'rb'), delimiter=' ', quotechar='|')
#spamReader = csv.reader(open('data/normal_spacehall/spacehall1154.vicon_velo.nk.data', 'rb'), delimiter=' ', quotechar='|')


timevicon=[]
velviconx=[]
velvicony=[]
velviconz=[]

for row in spamReader:
    #print row
    timevicon.append(float(row[0])/1000000)
    velviconx.append(float(row[1]))
    velvicony.append(float(row[2]))
    velviconz.append(float(row[3]))

deltavicon = []
for i in range(0,len(timevicon)-1):
    #print time[i]
    tvicon = float(timevicon[i+1]) - float(timevicon[i])
    deltavicon.append(tvicon)
    
deltavicon_t = mean(deltavicon)    
sample_ratevicon = 1/deltavicon_t
tvicon = deltavicon_t * r_[0:len(timevicon)]


#Load Velocity error
spamReader = csv.reader(open('data/normal_spacehall/spacehall1140.velocity_error.slip_gp.5.data', 'rb'), delimiter=' ', quotechar='|')
#spamReader = csv.reader(open('data/normal_spacehall/spacehall1154.velocity_error.nk.data', 'rb'), delimiter=' ', quotechar='|')


timeerror=[]
velerrorx=[]
velerrory=[]
velerrorz=[]

for row in spamReader:
    #print row
    timeerror.append(float(row[0])/1000000)
    velerrorx.append(float(row[1]))
    velerrory.append(float(row[2]))
    velerrorz.append(float(row[3]))
    
    
deltaerror = []
for i in range(0,len(timeerror)-1):
    #print timebody[i]
    terror = float(timeerror[i+1]) - float(timeerror[i])
    deltaerror.append(terror)
    
deltaerror_t = pylab.mean(deltaerror)    
sample_rateerror = 1/deltaerror_t
terror = deltaerror_t * pylab.r_[0:len(timeerror)]

#Load Velocity error truth (Vicon system)
spamReader = csv.reader(open('data/normal_spacehall/spacehall1140.velocity_error.vicon.slip_gp.2.data', 'rb'), delimiter=' ', quotechar='|')
#spamReader = csv.reader(open('data/normal_spacehall/spacehall1154.velocity_error.nk.data', 'rb'), delimiter=' ', quotechar='|')


timetrutherror=[]
veltrutherrorx=[]
veltrutherrory=[]
veltrutherrorz=[]

for row in spamReader:
    #print row
    timetrutherror.append(float(row[0])/1000000)
    veltrutherrorx.append(float(row[1]))
    veltrutherrory.append(float(row[2]))
    veltrutherrorz.append(float(row[3]))
    
    
deltatrutherror = []
for i in range(0,len(timetrutherror)-1):
    #print timebody[i]
    ttrutherror = float(timetrutherror[i+1]) - float(timetrutherror[i])
    deltatrutherror.append(ttrutherror)
    
deltatrutherror_t = pylab.mean(deltatrutherror)    
sample_ratetrutherror = 1/deltatrutherror_t
ttrutherror = deltatrutherror_t * pylab.r_[0:len(timetrutherror)]

#Load slip detector
spamReader = csv.reader(open('data/normal_spacehall/spacehall1140.slip_detector.slip_gp.2.data', 'rb'), delimiter=' ', quotechar='|')
#spamReader = csv.reader(open('data/normal_spacehall/spacehall1154.velocity_error.nk.data', 'rb'), delimiter=' ', quotechar='|')


timeslip=[]
velslipx=[]
velslipy=[]
velslipz=[]

for row in spamReader:
    #print row
    timeslip.append(float(row[0])/1000000)
    velslipx.append(float(row[1]))
    velslipy.append(float(row[2]))
    velslipz.append(float(row[3]))
    
    
deltaslip = []
for i in range(0,len(timeslip)-1):
    #print timebody[i]
    tslip = float(timeslip[i+1]) - float(timeslip[i])
    deltaslip.append(tslip)
    
deltaslip_t = pylab.mean(deltaslip)    
sample_rateslip = 1/deltaslip_t
tslip = deltaslip_t * pylab.r_[0:len(timeslip)]

plt.figure(4)
plot(tvicon[0:len(tvicon)-6],velviconx[6:len(tvicon)], '-o',label="X ground truth velocity")
pylab.plot(tbody,velbodyx, '-o', label="X model velocity estimation")
pylab.plot(terror,velerrorx, '-o', label="X velocity error")
pylab.plot(ttrutherror,veltrutherrorx, '-o', label="X truth velocity error")
pylab.plot(tslip,velslipx, '-o', label="X velocity error")
pylab.grid(True)
xlabel("Time(s)")
ylabel("Velocity(m/s)")
title("Asguard Body Velocity - Slip Path")
legend()
plt.show()

pylab.plot(tbody,velbodyx, marker='o', linestyle='--', label="X model velocity estimation", linewidth=2.5, color='black')

plt.figure(2)
pylab.plot(tbody,velbodyy, '-o', label="Y model velocity estimation")
pylab.plot(terror,velerrory, '-o', label="Y velocity error")
pylab.plot(tslip,velslipy, '-o', label="Y velocity error")
pylab.grid()
xlabel("Time(s)")
ylabel("Velocity(m/s)")
title("Asguard Body Velocity - Slip Path")
legend()
plt.show()


#AT 5HZ
plt.figure(5)
plot(tvicon,velviconx, '-o',label="X ground truth velocity")
pylab.plot(tbody,velbodyx, '-o', label="X model velocity estimation")
pylab.grid(True)
xlabel("Time(s)")
ylabel("Velocity(m/s)")
title("Asguard Body Velocity - Slip Path")
legend()
plt.show()


#100 and 5HZ COMPARISON
plt.figure(6)
pylab.plot(timebody,velbodyx, '-o', label="X model velocity estimation")
pylab.grid(True)
xlabel("Time(s)")
ylabel("Velocity(m/s)")
title("Asguard Body Velocity - Slip Path")
legend()
plt.show()

def predict_values(X, f, K, file_tag, learn=False):
    """
    Create a GP with kernel K and predict values.
    Optionally learn K's hyperparameters if learn==True.
    """
    gp = infpy.gp.GaussianProcess(X, f, K)
    if learn:
        infpy.gp.gp_learn_hyperparameters(gp)
    pylab.figure()
    infpy.gp.gp_1D_predict(gp, 90, x_min - 10., x_max + 10.)
    save_fig(file_tag)
    #pylab.close()
    return (gp)


#Rename the variables
def recu (A,B):
    gaussianpro = []
    for i in A:
	X2=[]
	for j in B:
	    X2.append([terror[i]])
	f2=numpy.array(velerrorx[3000:4000])


    K = kernels.SE([4.0]) + kernels.Noise(.25) # Learn kernel hyper-parameters
    gaussianpro.append(predict_values(X2, f2, K,'velocity-error-xaxis-learn'+str(j), learn=True))
    j = j+10
    


#Rename the variables
X3=[]
for i in range(2500,3500):
    X3.append([terror[i]])
    
f3=numpy.array(velerrorx[2500:3500])

K = kernels.SE([1.0]) + kernels.Noise(.15) # Learn kernel hyper-parameters
predict_values(X3, f3, K,'velocity-error-xaxis', learn=True)


#Sample at 2Hz or when value is bigger than sigma = 0.02
X3=[]
Y3=[]
j=0
sigma=1.92e-03
for i in range(0,len(terror)):
    if (j==(100/1)):
	X3.append([terror[i]])
	Y3.append(velerrorx[i])
	j=0
    j = j+1
    if (fabs(velerrorx[i-1]-velerrorx[i]) > 3*sigma):
	X3.append([terror[i]])
	Y3.append(velerrorx[i])
    


plt.figure(3)
pylab.plot(X3,Y3, '-o', label="X velocity error")
pylab.grid()
xlabel("Time(s)")
ylabel("Velocity(m/s)")
title("Asguard Body Velocity - Slip Path")
legend()
plt.show()
    
X3=[]
Y3=[]
j=0
sigma=1.92e-03
for i in range(0,len(terror)):
    if (fabs(velerrorx[i-1]-velerrorx[i]) > 0.05):
	X3.append([terror[i]])
	Y3.append(velerrorx[i])


K = kernels.SE([100.0]) + kernels.Noise(.15) # Learn kernel hyper-parameters
predict_values(X3[0:100], Y3[0:100], K,'velocity-error-xaxis', learn=True)
