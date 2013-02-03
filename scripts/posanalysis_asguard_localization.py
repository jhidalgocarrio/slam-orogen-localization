#!/usr/bin/python
import csv, scipy
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
#from scipy.integrate import integrate

#Original Data
#spamReader = csv.reader(open('data/multitest_spacehall/spacehall0940.puremodel_velo.incre.3.data', 'rb'), delimiter=' ', quotechar='|')
spamReader = csv.reader(open('data/multitest_spacehall/spacehall0940.puremodel_velo.3.data', 'rb'), delimiter=' ', quotechar='|')
#spamReader = csv.reader(open('data/multitest_spacehall/spacehall0940.corrected_velo.3.data', 'rb'), delimiter=' ', quotechar='|')

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


#IMU Data
spamReader = csv.reader(open('data/multitest_spacehall/spacehall0940.imu_acc_velo.incre.3.data', 'rb'), delimiter=' ', quotechar='|')
#spamReader = csv.reader(open('data/multitest_spacehall/spacehall0940.imu_acc_velo.0.data', 'rb'), delimiter=' ', quotechar='|')

timeimu=[]
velimux=[]
velimuy=[]
velimuz=[]
accimux=[]
accimuy=[]
accimuz=[]

for row in spamReader:
    #print row
    timeimu.append(float(row[0])/1000000)
    accimux.append(float(row[1]))
    accimuy.append(float(row[2]))
    accimuz.append(float(row[3]))
    velimux.append(float(row[4]))
    velimuy.append(float(row[5]))
    velimuz.append(float(row[6]))
    
    
    
deltaimu = []
for i in range(0,len(timeimu)-1):
    #print time[i]
    timu = float(timeimu[i+1]) - float(timeimu[i])
    deltaimu.append(timu)
    
deltaimu_t = mean(deltaimu)    
sample_rateimu = 1/deltaimu_t
timu = deltaimu_t * r_[0:len(timeimu)]

#Vicon data
spamReader = csv.reader(open('data/multitest_spacehall/spacehall0940.vicon_velo.0.data', 'rb'), delimiter=' ', quotechar='|')
#spamReader = csv.reader(open('data/multitest_spacehall/spacehall0940.vicon_velo.body_frame.0.data', 'rb'), delimiter=' ', quotechar='|')
#spamReader = csv.reader(open('data/multitest_spacehall/spacehall0940.vicon_velo.incre.3.data', 'rb'), delimiter=' ', quotechar='|')

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


#Velocities error Data
spamReader = csv.reader(open('data/multitest_spacehall/spacehall0940.error_velo.0.data', 'rb'), delimiter=' ', quotechar='|')

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
    
deltaerror_t = mean(deltaerror)    
sample_rateerror = 1/deltaerror_t
terror = deltaerror_t * r_[0:len(timeerror)]

#Incremental velocity data
spamReader = csv.reader(open('data/multitest_spacehall/spacehall0940.imu_acc_velo.incre.2.data', 'rb'), delimiter=' ', quotechar='|')

timeiimu=[]
veliimux=[]
veliimuy=[]
veliimuz=[]
acciimux=[]
acciimuy=[]
acciimuz=[]

for row in spamReader:
    #print row
    timeiimu.append(float(row[0])/1000000)
    acciimux.append(float(row[1]))
    acciimuy.append(float(row[2]))
    acciimuz.append(float(row[3]))
    veliimux.append(float(row[4]))
    veliimuy.append(float(row[5]))
    veliimuz.append(float(row[6]))
    
    
    
deltaiimu = []
for i in range(0,len(timeiimu)-1):
    #print time[i]
    tiimu = float(timeiimu[i+1]) - float(timeiimu[i])
    deltaiimu.append(tiimu)
    
deltaiimu_t = mean(deltaiimu)    
sample_rateiimu = 1/deltaiimu_t
tiimu = deltaiimu_t * r_[0:len(timeiimu)]


###############################
# Combine model+incremental
###############################
velcimux=[]
offset = 63
velcimux.append((velbodyx[0])-veliimux[0])
for i in range(1,len(timeiimu)-offset):
    error = float((velbodyx[i] - velbodyx[i-1])-veliimux[i])
    velcomb = float(velbodyx[i] - error)
    velcimux.append(velcomb)
    
velcimux=[]
offset = 63
velcimux.append(veliimux[0])
for i in range(1,len(timeiimu)-offset):
    velcomb = float(velbodyx[i-1]+veliimux[i])
    velcimux.append(velcomb)
    
velibodyx=[]
velibodyx.append(velbodyx[0])
for i in range(1,len(timebody)):
    velibodyx.append(velbodyx[i]-velbodyx[i-1])


#Filter Info Data
spamReader = csv.reader(open('data/multitest_spacehall/spacehall0940.filter_info.1.data', 'rb'), delimiter=' ', quotechar='|')

timefiltererror=[]
filtervelerrorx=[]
filtervelerrory=[]
filtervelerrorz=[]

for row in spamReader:
    #print row
    timefiltererror.append(float(row[0])/1000000)
    filtervelerrorx.append(float(row[4]))
    filtervelerrory.append(float(row[5]))
    filtervelerrorz.append(float(row[6]))
    
    
deltafiltererror = []
for i in range(0,len(timefiltererror)-1):
    #print timebody[i]
    tfiltererror = float(timefiltererror[i+1]) - float(timefiltererror[i])
    deltafiltererror.append(tfiltererror)
    
deltafiltererror_t = mean(deltafiltererror)    
sample_ratefiltererror = 1/deltafiltererror_t
tfiltererror = deltafiltererror_t * r_[0:len(timefiltererror)]


#################
### GRAPHICS  ###
#################
plt.figure(4)
plot(timebody,velbodyx, '-o', label="X model velocity estimation")
plot(timebody,np.cumsum(velbodyx), '-o', label="X model cum velocity estimation")
#plot(timebody,velibodyx, '-o', label="X model incremental velocity")
plot(timeimu,velimux, '-o', label="X imu incre velocity")
plot(timeimu,np.cumsum(velimux), '-o', label="X imu velocity")

#plot(timeiimu,veliimux, '-o', label="X imu incremental velocity")
#plot(timebody[0:len(timeiimu)-63],veliimux[63:len(timeiimu)], '-o', label="X imu incremental velocity")
#plot(timebody[0:len(timeiimu)-63],velcimux, '-o', label="X combined imu velocity")
plot(timeimu,accimux, '-o', label="X imu acc(2)")
#plot(timeiimu,acciimux, '-o', label="X imu acc (incremental log)")
plot(timevicon[0:len(timevicon)-6],velviconx[6:len(timevicon)], '-o',label="X ground truth velocity")
plot(timeerror,velerrorx, '-o', label="X error velocity")
plot(timefiltererror,filtervelerrorx, '-o', label="X filter error velocity")
grid()
xlabel("Time(s)")
ylabel("Velocity(m/s)")
title("Asguard Body Center Velocity")
legend()


fig = plt.figure(20)
ax = fig.add_subplot(111)
model_vel = ax.plot(timebody,velbodyx, '-o', label="X model incre velocity estimation")
imu_vel = ax.plot(timeimu,velimux, '-o', label="X imu incre velocity")
vicon_vel = ax.plot(timevicon[0:len(timevicon)-6],velviconx[6:len(timevicon)], '-o',label="X ground truth velocity")
ax2 = ax.twinx()
imu_acc = ax2.plot(timeimu,accimux, '-o', label="X imu acc", color='m')

lns = model_vel+imu_vel+vicon_vel+imu_acc
labs = [l.get_label() for l in lns]
ax.legend(lns, labs, loc=0)

ax.grid()
ax.set_xlabel("Time(s)")
ax.set_ylabel("Velocity(m/s)")
ax.set_ylim(-10, 10)
ax2.set_ylabel("Acc (m/s^2)")
ax2.set_ylim(-30, 30)
plt.show()


plt.figure(3)
plot(timebody,velbodyy, '-o', label="Y model velocity")
plot(timeimu,velimuy, '-o', label="Y imu velocity")
plot(timeimu,accimuy, '-o', label="Y imu acc")
plot(timevicon[0:len(timevicon)-6],velvicony[6:len(timevicon)], '-o', label="Y vicon velocity")
plot(timeerror,velerrory, '-o', label="Y error velocity")
grid()
xlabel("Time(s)")
ylabel("Velocity(m/s)")
title("Body versus IMU velocity")
legend()


plt.figure(4)
plot(timebody,velbodyz, '-o', label="Z model velocity")
plot(timeimu,velimuz, '-o', label="Z imu velocity")
plot(timeimu,accimuz, '-o', label="Z imu acc")
#plot(timeiimu,acciimuz, '-o', label="Z imu acc")
plot(timevicon[0:len(timevicon)-6],velviconz[6:len(timevicon)], '-o', label="Z vicon velocity")
plot(timeerror,velerrorz, '-o', label="Z error velocity")
plot(timefiltererror,filtervelerrorz, '-o', label="Z filter error velocity")
grid()
xlabel("Time(s)")
ylabel("Velocity(m/s)")
title("Body versus IMU velocity")
legend()



### SUM BY BLOCK ##

blocksize = 100

#For Model incre
velblockbodyx = []
timeblockbody = []
count = 0    

while (count < len(velbodyx)):
    print np.sum(velbodyx[count-blocksize:count])
    velblockbodyx.append(np.sum(velbodyx[count-blocksize:count]))
    timeblockbody.append(timebody[count])
    count = count + blocksize





#For IMU incre
velblockimux = []
timeblockimu = []
count = 0    

while (count < len(velimux)):
    print np.sum(velimux[count-blocksize:count])
    velblockimux.append(np.sum(velimux[count-blocksize:count]))
    timeblockimu.append(timeimu[count])
    count = count + blocksize




#For vicon incre
velblockviconx = []
timeblockvicon = []
count = 0    

while (count < len(velviconx)):
    print np.sum(velviconx[count-blocksize:count])
    velblockviconx.append(np.sum(velviconx[count-blocksize:count]))
    timeblockvicon.append(timevicon[count])
    count = count + blocksize

plt.figure(7)
plot(timeblockbody,velblockbodyx, '-o', label="X model block sum velocity estimation")
plot(timeblockimu,velblockimux, '-o', label="X imu block sum velocity estimation")
plot(timeblockvicon,velblockviconx, '-o', label="X vicon block sum velocity estimation")
grid()
xlabel("Time(s)")
ylabel("Velocity(m/s)")
title("Asguard Body Center Velocity(1Hz)")
legend()

####################
# 6th order deriva
####################
veli6bodyx = []
acci6bodyx =[]
timei6body = []
count = 6

for i in range(0,count):
  veli6bodyx.append(velbodyx[i])
  acci6bodyx.append(0.00)
  timei6body.append(timebody[i])

while (count < len(velbodyx)):
    aux = (2.45*velbodyx[count] - 6*velbodyx[count-1] + 7.5*velbodyx[count-2] - (20.0/3.0)*velbodyx[count-3]
		      + 3.75*velbodyx[count-4] - 1.2*velbodyx[count-5] + (1.0/6.0)*velbodyx[count-6])
    veli6bodyx.append(aux/6.0)
    acci6bodyx.append(aux/(1.0/100.0))
    print veli6bodyx[count]
    timei6body.append(timebody[count])
    count = count + 1


plt.figure(8)
plot(timebody,velbodyx, '-o', label="X model incre velocity estimation(task)")
plot(timei6body,veli6bodyx, '-o', label="X model incre velocity estimation")
plot(timeimu,velimux, '-o', label="X imu incre velocity estimation")
plot(timevicon,velviconx, '-o', label="X vicon velocity estimation")
grid()
xlabel("Time(s)")
ylabel("Velocity(m/s)")
title("Asguard Body Center Velocity")
legend()


plt.figure(9)
plot(timei6body,acci6bodyx, '-o', label="X model acc estimation")
plot(timeimu,accimux, '-o', label="X imu acc")
plot(timevicon,velviconx, '-o', label="X vicon velocity estimation")
grid()
xlabel("Time(s)")
ylabel("Velocity(m/s)")
title("Asguard Body Center Acceleration")
legend()

##################################
# 1stOrder Filter Incre VeloModel
##################################
velfilterbodyx=[]
delta_t=1.0/100.0
Tc=1.0/16.0

velfilterbodyx.append(velbodyx[0])

for i in range(1,len(velbodyx)):
  velfilterbodyx.append(velfilterbodyx[i-1] + (delta_t/Tc)*(velbodyx[i]-velfilterbodyx[i-1]))
  
##################################
# 1stOrder Filter Incre Vicon
##################################
velfilterviconx=[]
delta_t=1.0/100.0
Tc=1.0/16.0

velfilterviconx.append(velviconx[0])

for i in range(1,len(velviconx)):
  velfilterviconx.append(velfilterviconx[i-1] + (delta_t/Tc)*(velviconx[i]-velfilterviconx[i-1]))


velmeanbodyx=[] 
#combine filter velo mode with imu
for i in range(0,len(timebody)):
  velmeanbodyx.append((velfilterbodyx[i]+velimux[i])/2.0)
  
  
plt.figure(10)
#plot(timebody,velbodyx, '-o', label="X model incre velocity estimation(task)")
plot(timebody,velfilterbodyx, '-o', label="X model incre velocity estimation(low-pass filter)")
plot(timeimu,velimux, '-o', label="X imu incre velocity estimation")
plot(timeimu,velmeanbodyx, '-o', label="X mean velocity estimation")
plot(timeimu,np.cumsum(velmeanbodyx), '-o', label="X mean cumsum velocity estimation")
plot(timevicon,velviconx, '-o', label="X vicon velocity estimation")
plot(timevicon[0:len(timevicon)-6],velfilterviconx[6:len(timevicon)], '-o', label="X vicon velocity estimation(low-pass filter)")
grid()
xlabel("Time(s)")
ylabel("Velocity(m/s)")
title("Asguard Body Center Velocity")
legend()

plt.figure(11)
plot(timebody,velbodyx, '-o', label="X model velocity estimation(task)")
plot(timebody,velfilterbodyx, '-o', label="X model incre velocity estimation(low-pass filter)")
plot(timebody,np.cumsum(velfilterbodyx), '-o', label="X model cumsum velocity estimation(low-pass filter)")
plot(timeimu,velimux, '-o', label="X imu incre velocity estimation")
plot(timeimu,np.cumsum(velimux), '-o', label="X imu cumsum velocity")
plot(timevicon[0:len(timevicon)-6],velviconx[6:len(timevicon)], '-o',label="X ground truth velocity")
plot(timevicon,velfilterviconx, '-o', label="X vicon incre velocity estimation(low-pass filter)")
plot(timevicon,np.cumsum(velfilterviconx), '-o', label="X vicon cumsum velocity estimation(low-pass filter)")
grid()
xlabel("Time(s)")
ylabel("Velocity(m/s)")
title("Asguard Body Center Velocity")
legend()




####################
# Position
####################
spamReader = csv.reader(open('data/multitest_spacehall/spacehall0940.puremodel_position.1.data', 'rb'), delimiter=' ', quotechar='|')


timemodel=[]
posmodelx=[]
posmodely=[]
posmodelz=[]

for row in spamReader:
    #print row
    timemodel.append(float(row[0])/1000000)
    posmodelx.append(float(row[1]))
    posmodely.append(float(row[2]))
    posmodelz.append(float(row[3]))


spamReader = csv.reader(open('data/multitest_spacehall/spacehall0940.puremodel_position.center.2.data', 'rb'), delimiter=' ', quotechar='|')


timebody=[]
posbodyx=[]
posbodyy=[]
posbodyz=[]

for row in spamReader:
    #print row
    timebody.append(float(row[0])/1000000)
    posbodyx.append(float(row[1]))
    posbodyy.append(float(row[2]))
    posbodyz.append(float(row[3]))


spamReader = csv.reader(open('data/multitest_spacehall/spacehall0940.vicon_position.1.data', 'rb'), delimiter=' ', quotechar='|')


timevicon=[]
posviconx=[]
posvicony=[]
posviconz=[]

for row in spamReader:
    #print row
    timevicon.append(float(row[0])/1000000)
    posviconx.append(float(row[1]))
    posvicony.append(float(row[2]))
    posviconz.append(float(row[3]))


#################
### GRAPHICS  ###
#################
plt.figure(2)
plot(timemodel,posmodelx, label="X model position(roverlocal)")
plot(timebody,posbodyx, label="X body position")
#plot(timu,accimux, label="X imu acc")
plot(timevicon,posviconx, label="X vicon position")
grid()
xlabel("Time(s)")
ylabel("Position(m)")
legend()


plt.figure(3)
plot(timemodel,posmodely, label="Y model position")
#plot(timebody,posbodyy, label="Y body position")
#plot(timu,accimux, label="X imu acc")
plot(timevicon,posvicony, label="Y vicon position")
grid()
xlabel("Time(s)")
ylabel("Position(m)")
legend()


plt.figure(4)
plot(timemodel,posmodelz, label="Z model position(roverlocal)")
plot(timebody,posbodyz, label="Z body position")
#plot(timu,accimux, label="X imu acc")
plot(timevicon,posviconz, label="Z vicon position")
grid()
xlabel("Time(s)")
ylabel("Position(m)")
legend()



plt.figure(5)
plot(posmodelx,posmodely, label="Model with dynamic wheel-weithing matrix")
plot(posbodyx,posbodyy, label="Model without wheel-weithing matrix")
#plot(timu,accimux, label="X imu acc")
plot(posviconx,posvicony, label="Vicon position")
grid()
xlabel("Position X-axis (m)")
ylabel("Position Y-axis (m)")
legend()

####################
# Inertial sensors
####################
spamReader = csv.reader(open('/home/jhidalgocarrio/iMoby/experiments/20121109_multitest_spacehall/20121109-0940/data/stim300.acc.0.data', 'rb'), delimiter=' ', quotechar='|')


time=[]
accx=[]
accy=[]
accz=[]

for row in spamReader:
    #print row
    time.append(float(row[0])/1000000)
    accx.append(float(row[1]))
    accy.append(float(row[2]))
    accz.append(float(row[3]))
    
    
delta = []
for i in range(0,len(time)-1):
    #print time[i]
    t = float(time[i+1]) - float(time[i])
    delta.append(t)
    
delta_t = mean(delta)    
sample_rate = 1/delta_t
t = delta_t * r_[0:len(time)]


meanaccx = []
stdaccxpos = []
stdaccxneg = []
numbermean = mean(accx[100:28000])
accx = accx - mean(accx[100:28000])
numberstd = std(accx[100:28000])
numbervar = var(accx[100:28000])
for i in range(0,len(time[100:28000])):
    meanaccx.append(numbermean)
    stdaccxpos.append(numberstd)
    stdaccxneg.append(-numberstd)

plt.figure(3)
plot(t[100:28000],accx[100:28000], label="X axis at 16Hz")
plot(t[100:28000],meanaccx, label="Mean value")
plot(t[100:28000],stdaccxpos, label="Std (+)")
plot(t[100:28000],stdaccxneg, label="Std (-)")



spamReader = csv.reader(open('/home/jhidalgocarrio/iMoby/experiments/20121109_multitest_spacehall/20121109-0940/data/stim300.gyro.0.data', 'rb'), delimiter=' ', quotechar='|')


time=[]
gyrox=[]
gyroy=[]
gyroz=[]

for row in spamReader:
    #print row
    time.append(float(row[0])/1000000)
    gyrox.append(float(row[1]))
    gyroy.append(float(row[2]))
    gyroz.append(float(row[3]))
    
    
delta = []
for i in range(0,len(time)-1):
    #print time[i]
    t = float(time[i+1]) - float(time[i])
    delta.append(t)
    
delta_t = mean(delta)    
sample_rate = 1/delta_t
t = delta_t * r_[0:len(time)]


meangyrox = []
stdgyroxpos = []
stdgyroxneg = []
numbermean = mean(gyrox[100:28000])
numberstd = std(gyrox[100:28000])
numbervar = var(gyrox[100:28000])
for i in range(0,len(time[100:28000])):
    meangyrox.append(numbermean)
    stdgyroxpos.append(numberstd)
    stdgyroxneg.append(-numberstd)

plt.figure(3)
plot(t[100:28000],gyrox[100:28000], label="X axis at 16Hz")
plot(t[100:28000],meangyrox, label="Mean value")
plot(t[100:28000],stdgyroxpos, label="Std (+)")
plot(t[100:28000],stdgyroxneg, label="Std (-)")

spamReader = csv.reader(open('data/multitest_spacehall/spacehall0940.imu_acc_velo.static_part.data', 'rb'), delimiter=' ', quotechar='|')

time=[]
velox=[]
veloy=[]
veloz=[]

for row in spamReader:
    #print row
    time.append(float(row[0])/1000000)
    velox.append(float(row[1]))
    veloy.append(float(row[2]))
    veloz.append(float(row[3]))
    
    
delta = []
for i in range(0,len(time)-1):
    #print time[i]
    t = float(time[i+1]) - float(time[i])
    delta.append(t)
    
delta_t = mean(delta)    
sample_rate = 1/delta_t
t = delta_t * r_[0:len(time)]


meanvelox = []
stdveloxpos = []
stdveloxneg = []
numbermean = mean(velox)
numberstd = std(velox)
numbervar = var(velox)
for i in range(0,len(time)):
    meanvelox.append(numbermean)
    stdveloxpos.append(numberstd)
    stdveloxneg.append(-numberstd)

plt.figure(3)
plot(t,velox, label="X axis at 16Hz bandwidth(125Hz sampling)")
plot(t,meanvelox, label="Mean value")
plot(t,stdveloxpos, label="Std (+)")
plot(t,stdveloxneg, label="Std (-)")
xlabel("Time(s)")
ylabel("Velocity(m/s)")
grid()
legend()


meanveloy = []
stdveloypos = []
stdveloyneg = []
numbermean = mean(veloy)
numberstd = std(veloy)
numbervar = var(veloy)
for i in range(0,len(time)):
    meanveloy.append(numbermean)
    stdveloypos.append(numberstd)
    stdveloyneg.append(-numberstd)

plt.figure(4)
plot(t,veloy, label="Y axis at 16Hz bandwidth(125Hz sampling)")
plot(t,meanveloy, label="Mean value")
plot(t,stdveloypos, label="Std (+)")
plot(t,stdveloyneg, label="Std (-)")
xlabel("Time(s)")
ylabel("Velocity(m/s)")
grid()
legend()


meanveloz = []
stdvelozpos = []
stdvelozneg = []
numbermean = mean(veloz)
numberstd = std(veloz)
numbervar = var(veloz)
for i in range(0,len(time)):
    meanveloz.append(numbermean)
    stdvelozpos.append(numberstd)
    stdvelozneg.append(-numberstd)

plt.figure(5)
plot(t,veloz, label="Z axis at 16Hz bandwidth(125Hz sampling)")
plot(t,meanveloz, label="Mean value")
plot(t,stdvelozpos, label="Std (+)")
plot(t,stdvelozneg, label="Std (-)")
xlabel("Time(s)")
ylabel("Velocity(m/s)")
grid()
legend()

######################
# Inertial Acc sensors
######################

scale = [2, 5, 10, 30 ,50, 100, 200]
bias_stability = [1.5, 3.75, 7.5, 22, 37.5, 75, 150]

plt.figure(2)
plot(scale,bias_stability, '-o', label="Colibrys Acc")
grid()
xlabel("Accelerometers Range (g)")
ylabel("Bias instability(mg)")
title("Accelerometers Range versus Bias instability")
legend()


##########################
# Inertial sensor I/O Port
##########################

#IMU Input Data 
#spamReader = csv.reader(open('data/slip_test_white_room/whiteroom1616.imu_inport.0.data', 'rb'), delimiter=' ', quotechar='|')
spamReader = csv.reader(open('data/multitest_spacehall/spacehall0940.imu_inport.3.data', 'rb'), delimiter=' ', quotechar='|')

timeimuin=[]
accimuinx=[]
accimuiny=[]
accimuinz=[]
gyroimuinx=[]
gyroimuiny=[]
gyroimuinz=[]

for row in spamReader:
    #print row
    timeimuin.append(float(row[0])/1000000)
    accimuinx.append(float(row[1]))
    accimuiny.append(float(row[2]))
    accimuinz.append(float(row[3]))
    gyroimuinx.append(float(row[4]))
    gyroimuiny.append(float(row[5]))
    gyroimuinz.append(float(row[6]))
    
    
    
    
#IMU Output Data 
#spamReader = csv.reader(open('data/slip_test_white_room/whiteroom1616.imu_outport.0.data', 'rb'), delimiter=' ', quotechar='|')
spamReader = csv.reader(open('data/multitest_spacehall/spacehall0940.imu_outport.3.data', 'rb'), delimiter=' ', quotechar='|')

timeimuout=[]
accimuoutx=[]
accimuouty=[]
accimuoutz=[]
gyroimuoutx=[]
gyroimuouty=[]
gyroimuoutz=[]

for row in spamReader:
    #print row
    timeimuout.append(float(row[0])/1000000)
    accimuoutx.append(float(row[1]))
    accimuouty.append(float(row[2]))
    accimuoutz.append(float(row[3]))
    gyroimuoutx.append(float(row[4]))
    gyroimuouty.append(float(row[5]))
    gyroimuoutz.append(float(row[6]))

plt.figure(2)
plot(timeimuin,accimuinx, '-o', label="Acc X-axis Inport")
plot(timeimuout,accimuoutx, '-o', label="Acc X-axis Outport")
grid()
xlabel("Time(s)")
ylabel("Acc(m/s^2)")
legend()


plt.figure(2)
plot(timeimuin, gyroimuinx, '-o', label="Gyro X-axis Inport")
plot(timeimuout,gyroimuoutx, '-o', label="Gyro X-axis Outport")
grid()
xlabel("Time(s)")
ylabel("Angular velocity(m/s^2)")
legend()
