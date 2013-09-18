from random import gauss

#FrontEnd reference from Ground Truth
frontendreferencestim300 = ThreeData()
frontendreferencestim300.readData('data/normal_spacehall/frontend_referencepose_velocity.1154.stim300.0.data', cov=True)
frontendreferencestim300.eigenValues()

#FrontEnd Motion model velocity
frontendbodystim300 = ThreeData()
frontendbodystim300.readData('data/normal_spacehall/frontend_poseout_velocity.1154.stim300.0.data', cov=True)
frontendbodystim300.eigenValues()

frontendbody015 = ThreeData()
frontendbody015.readData('data/normal_spacehall/frontend_poseout_velocity.1154.0.15.0.data', cov=True)
frontendbody015.eigenValues()

frontendbody05 = ThreeData()
frontendbody05.readData('data/normal_spacehall/frontend_poseout_velocity.1154.0.5.0.data', cov=True)
frontendbody05.eigenValues()

frontendbody10 = ThreeData()
frontendbody10.readData('data/normal_spacehall/frontend_poseout_velocity.1154.1.0.1.data', cov=True)
frontendbody10.eigenValues()

frontendbody20 = ThreeData()
frontendbody20.readData('data/normal_spacehall/frontend_poseout_velocity.1154.2.0.0.data', cov=True)
frontendbody20.eigenValues()

frontendbody30 = ThreeData()
frontendbody30.readData('data/normal_spacehall/frontend_poseout_velocity.1154.3.0.0.data', cov=True)
frontendbody30.eigenValues()

frontendbody40 = ThreeData()
frontendbody40.readData('data/normal_spacehall/frontend_poseout_velocity.1154.4.0.0.data', cov=True)
frontendbody40.eigenValues()

frontendbody50 = ThreeData()
frontendbody50.readData('data/normal_spacehall/frontend_poseout_velocity.1154.5.0.0.data', cov=True)
frontendbody50.eigenValues()

frontendbodyxsens = ThreeData()
frontendbodyxsens.readData('data/normal_spacehall/frontend_poseout_velocity.1154.xsens.0.data', cov=True)
frontendbodyxsens.eigenValues()

frontendreferencestim300.plot_axis(1, 0,False, 1, True, [1,0,0])
frontendbodystim300.plot_axis(1, 0,True, 1, True, [0,0,1])
frontendbody015.plot_axis(1, 0,True, 1, True, [0,0,1])
frontendbody05.plot_axis(1, 0,True, 1, True, [0,0,0])
frontendbody10.plot_axis(1, 0,True, 1, True, [0,0.2,1])
frontendbody20.plot_axis(1, 0,True, 1, True, [0,0.4,1])
frontendbody30.plot_axis(1, 0,True, 1, True, [1,0.6,1])
frontendbody40.plot_axis(1, 0,True, 1, True, [0,0.8,1])
frontendbodyxsens.plot_axis(1, 0,True, 1, True, [0,1,1])

frontendbodystim300.plot_axis(1, 1,True, 1, True, [0,0,1])
frontendbody10.plot_axis(1, 1,True, 1, True, [0,0.2,1])
frontendbody40.plot_axis(1, 1,True, 1, True, [0,0.8,1])
frontendbodyxsens.plot_axis(1, 1,True, 1, True, [0,1,1])

frontendbody10.plot_axis(1, 2,True, 1, True, [0,0.2,1])
frontendbody40.plot_axis(1, 2,True, 1, True, [0,0.8,1])
frontendbodyxsens.plot_axis(1, 2,True, 1, True, [0,1,1])

#Analysis

varxstim300 = []
varystim300 = []
varzstim300 = []
sum(frontendbodystim300.var)
for i in range(0,len(frontendbodystim300.var)):
	varxstim300.append(frontendbodystim300.var[i][0])
	varystim300.append(frontendbodystim300.var[i][1])
	varzstim300.append(frontendbodystim300.var[i][2])

varx015 = []
vary015 = []
varz015 = []
sum(frontendbody015.var)
for i in range(0,len(frontendbody015.var)):
	varx015.append(frontendbody015.var[i][0])
	vary015.append(frontendbody015.var[i][1])
	varz015.append(frontendbody015.var[i][2])

varx05 = []
vary05 = []
varz05 = []
sum(frontendbody05.var)
for i in range(0,len(frontendbody05.var)):
	varx05.append(frontendbody05.var[i][0])
	vary05.append(frontendbody05.var[i][1])
	varz05.append(frontendbody05.var[i][2])


varx10 = []
vary10 = []
varz10 = []
sum(frontendbody10.var)
for i in range(0,len(frontendbody10.var)):
	varx10.append(frontendbody10.var[i][0])
	vary10.append(frontendbody10.var[i][1])
	varz10.append(frontendbody10.var[i][2])

varx20 = []
vary20 = []
varz20 = []
sum(frontendbody20.var)
for i in range(0,len(frontendbody20.var)):
	varx20.append(frontendbody20.var[i][0])
	vary20.append(frontendbody20.var[i][1])
	varz20.append(frontendbody20.var[i][2])

varx30 = []
vary30 = []
varz30 = []
sum(frontendbody30.var)
for i in range(0,len(frontendbody30.var)):
	varx30.append(frontendbody30.var[i][0])
	vary30.append(frontendbody30.var[i][1])
	varz30.append(frontendbody30.var[i][2])

varx40 = []
vary40 = []
varz40 = []
sum(frontendbody40.var)
for i in range(0,len(frontendbody40.var)):
	varx40.append(frontendbody40.var[i][0])
	vary40.append(frontendbody40.var[i][1])
	varz40.append(frontendbody40.var[i][2])

varx50 = []
vary50 = []
varz50 = []
sum(frontendbody50.var)
for i in range(0,len(frontendbody50.var)):
	varx50.append(frontendbody50.var[i][0])
	vary50.append(frontendbody50.var[i][1])
	varz50.append(frontendbody50.var[i][2])


varxxsens = []
varyxsens = []
varzxsens = []
sum(frontendbodyxsens.var)
for i in range(0,len(frontendbodyxsens.var)):
	varxxsens.append(frontendbodyxsens.var[i][0])
	varyxsens.append(frontendbodyxsens.var[i][1])
	varzxsens.append(frontendbodyxsens.var[i][2])



#Plotting
stdvectorx = [mean(sqrt(varx015)), mean(sqrt(varx05)), mean(sqrt(varx10)),  mean(sqrt(varx20)),  mean(sqrt(varx30)),  mean(sqrt(varx40)), mean(sqrt(varx50))]
stdvectory = [mean(sqrt(vary015)),   mean(sqrt(vary05)), mean(sqrt(vary10)),  mean(sqrt(vary20)),  mean(sqrt(vary30)),  mean(sqrt(vary40)), mean(sqrt(vary50))]
stdvectorz = [mean(sqrt(varz015)),   mean(sqrt(varz05)), mean(sqrt(varz10)),  mean(sqrt(varz20)),  mean(sqrt(varz30)),  mean(sqrt(varz40)), mean(sqrt(varz50))]
randomwalk = [0.15, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0]

fig  = plt.figure(1)
ax = fig.add_subplot(111)
plt.plot(randomwalk, stdvectorx, marker='D', markersize=10, label="Body x-axis", color=[1,0,0], linestyle='--', lw=2 )
plt.plot(randomwalk, stdvectory, marker='v', markersize=10, label="Body y-axis", color=[0,1,0], linestyle='--', lw=2 )
plt.plot(randomwalk, stdvectorz, marker='*', markersize=10, label="Body z-axis", color=[0,0,1], linestyle='--', lw=2 )
grid()
plt.ylabel(r'Standard deviation [$m/s$]')
plt.xlabel(r'Angular random walk [${\degree}/{\sqrt{h}} $]')
plt.grid(True)
plt.legend(prop={'size':30})
plt.show(block=False)
savefig('figures/gyros_analysis_moving.png')


plot(frontendbody015.t, varx015 )
plot(frontendbody40.t, varx40 )

grid()
plt.show(block=False)

valuesstim300 = []
values10 = []
values20 = []
values30 = []
values40 = []
while len(valuesstim300) < 1000:
	value = gauss(mean(sqrt(varxstim300)), std(sqrt(varxstim300)))
	valuesstim300.append(value)
	value = gauss(mean(sqrt(varx10)), std(sqrt(varx10)))
	values10.append(value)
	value = gauss(mean(sqrt(varx20)), std(sqrt(varx20)))
	values20.append(value)
	value = gauss(mean(sqrt(varx30)), std(sqrt(varx30)))
	values30.append(value)
	value = gauss(mean(sqrt(varx40)), std(sqrt(varx40)))
	values40.append(value)

values =  [np.array(valuesstim300), np.array(values10), np.array(values20), np.array(values30), np.array(values40)]

figure()
boxplot(values)
show(block=False)

#Ploting the begining of the velocity
plt.figure(1)
values = frontendbody30.getAxis(0)[0:2000]
plt.plot(frontendbody30.t[0:2000], values,
        marker='.', label="Motion Model X-axis", color=[0,0,1], lw=2)
plt.ylabel(r'Velocity [$m/s$]',  fontsize=14)
plt.xlabel(r'Time [$s$]',  fontsize=14)
plt.grid(True)
plt.legend(prop={'size':15})
plt.show(block=False)


# Create some data
spread= rand(50) * 100
center = ones(25) * 40
flier_high = rand(10) * 100 + 100
flier_low = rand(10) * -100
d2 = concatenate( (spread, center, flier_high, flier_low), 0 )
data.shape = (-1, 1)
d2.shape = (-1, 1)
#data = concatenate( (data, d2), 1 )
# Making a 2-D array only works if all the columns are the
# same length.  If they are not, then use a list instead.
# This is actually more efficient because boxplot converts
# a 2-D array into a list of vectors internally anyway.
data = [data, d2, d2[::2,0]]
# multiple box plots on one figure
figure()
boxplot(data)
#savefig('box7')

show()

random.gauss(0, 10)


import numpy as np 
import pylab 
import scipy.stats as stats

stats.probplot(varxstim300, dist="norm", plot=pylab)
pylab.show(block=False)

#Chi Square test to the error in velocity (model-truth)
plt.figure(1)
values = frontendbody100Hz.getAxis(0)[5550:8000]
plt.plot(frontendbody100Hz.t[5550:8000], values,
        marker='.', label="Motion Model X-axis", color=[0,0,1], lw=2)
plt.plot(frontendbody100Hz.t[5550:8000], frontendbody100Hz.getStdMax(0)[5550:8000] , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 1\sigma$ uncertainty')
plt.plot(frontendbody100Hz.t[5550:8000], frontendbody100Hz.getStdMin(0)[5550:8000] , color=[0,0,0], linestyle='--', lw=2)
values=frontendreference100Hz.getAxis(0)[5550:8000]
plt.plot(frontendreference100Hz.t[5550:8000], values,
        marker='D', label="Ground Truth X-axis", color=[0,0.5,0.5], alpha=0.5, lw=5)
plt.ylabel(r'Velocity [$m/s$]',  fontsize=14)
plt.xlabel(r'Time [$s$]',  fontsize=14)
plt.grid(True)
plt.legend(prop={'size':15})
plt.show(block=False)

T = []
for i in range(0,len(frontendbody100Hz.data)):
    T.append(np.dot(frontendbody100Hz.data[i] , np.dot(linalg.inv(frontendbody100Hz.cov[i]) , frontendbody100Hz.data[i].T)))


error = []
for i in range(0,len(frontendbody100Hz.data)):
	    values.append(self.data[i][axis])
            sdmax.append(values[i]+(sqrt(self.var[i][axis])))

values = np.array(model) - np.array(truth)

#Ploting Xsens and STIM300 motion model comparison

font = {'family' : 'sans-serif',
        'weight' : 'medium',
        'size'   : 25}

matplotlib.rc('font', **font)
matplotlib.rcParams.update({'font.size': 25})

plt.figure(1)
ax = fig.add_subplot(111)
values = frontendbodystim300.getAxis(0)[0:len(frontendbodystim300.t)-2]
plt.plot(frontendbodystim300.t[2:len(frontendbodystim300.t)], values,
        marker='.', label="Expected value with tactical-grade IMU", color=[0,0,1], lw=2)
plt.plot(frontendbodystim300.t[2:len(frontendbodystim300.t)], frontendbodystim300.getStdMax(0)[0:len(frontendbodystim300.t)-2], color=[0,0.5,1], linestyle='--', lw=2, label=r'$\pm 1\sigma$ uncertainty with tactical-grade IMU')
plt.plot(frontendbodystim300.t[2:len(frontendbodystim300.t)], frontendbodystim300.getStdMin(0)[0:len(frontendbodystim300.t)-2], color=[0,0.5,1], linestyle='--', lw=2)
values = frontendbodyxsens.getAxis(0)[0:len(frontendbodyxsens.t)-40]
plt.plot(frontendbodyxsens.t[40:len(frontendbodyxsens.t)], values,
        marker='*', label="Expected value with consumer-grade IMU", color=[1,0,0], lw=2)
plt.plot(frontendbodyxsens.t[40:len(frontendbodyxsens.t)], frontendbodyxsens.getStdMax(0)[0:len(frontendbodyxsens.t)-40], color=[1,0,0.5], linestyle='-.', lw=2.4, label=r'$\pm 1\sigma$ uncertainty with consumer-grade IMU')
plt.plot(frontendbodyxsens.t[40:len(frontendbodyxsens.t)], frontendbodyxsens.getStdMin(0)[0:len(frontendbodyxsens.t)-40], color=[1,0,0.5], linestyle='-.', lw=2.4)
values=frontendreferencestim300.getAxis(0)
plt.plot(frontendreferencestim300.t, values,
        marker='D', label="Ground Truth data", color=[0,0.5,0.5], alpha=0.5, lw=5)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/normalgrass_velocity_model_stim300_vs_xsens.png')


for i in range(0,len(frontendreferencestim300.data)):
    if frontendreferencestim300.data[i][0] > 0.6:
        frontendreferencestim300.data[i] = frontendreferencestim300.data[i-1]

#Delete outliers
frontendreferencestim300.data[4089][0] = 0.0296
frontendreferencestim300.data[4091][0] = 0.06
frontendreferencestim300.data[4092][0] = 0.06
np.where(frontendreferencestim300.t > [40.8])

for i in range(4348,len(frontendreferencestim300.data)):
    frontendreferencestim300.data[i][0] = 0.00


#Comparing uncertainties for different weighting matrix
#FrontEnd Motion model velocity
frontendbodyw025 = ThreeData()
frontendbodyw025.readData('data/normal_spacehall/weighting/frontend_poseout_velocity.1154.0.25.1.data', cov=True)
frontendbodyw025.eigenValues()

frontendbodyw1 = ThreeData()
frontendbodyw1.readData('data/normal_spacehall/weighting/frontend_poseout_velocity.1154.1.2.data', cov=True)
frontendbodyw1.eigenValues()

frontendbodyw1 = ThreeData()
frontendbodyw1.readData('data/normal_spacehall/weighting/frontend_poseout_velocity.1154.dynamic.0.data', cov=True)
frontendbodyw1.eigenValues()

plt.figure(1)
ax = fig.add_subplot(111)
values = frontendbodyw025.getAxis(0)[0:len(frontendbodyw025.t)-2]
plt.plot(frontendbodyw025.t[2:len(frontendbodyw025.t)], values,
        marker='.', label="Expected value with 0.25", color=[0,0,1], lw=2)
plt.plot(frontendbodyw025.t[2:len(frontendbodyw025.t)], frontendbodyw025.getStdMax(0)[0:len(frontendbodyw025.t)-2], color=[0,0.5,1], linestyle='--', lw=2, label=r'$\pm 1\sigma$ uncertainty with tactical-grade IMU')
plt.plot(frontendbodyw025.t[2:len(frontendbodyw025.t)], frontendbodyw025.getStdMin(0)[0:len(frontendbodyw025.t)-2], color=[0,0.5,1], linestyle='--', lw=2)
values = frontendbodyw1.getAxis(0)[0:len(frontendbodyw1.t)-2]
plt.plot(frontendbodyw1.t[2:len(frontendbodyw1.t)], values,
        marker='*', label="Expected value with consumer-grade IMU", color=[0,0,0], lw=2)
plt.plot(frontendbodyw1.t[2:len(frontendbodyw1.t)], frontendbodyw1.getStdMax(0)[0:len(frontendbodyw1.t)-2], color=[0,0,0.5], linestyle='-.', lw=2.4, label=r'$\pm 1\sigma$ uncertainty with consumer-grade IMU')
plt.plot(frontendbodyw1.t[2:len(frontendbodyw1.t)], frontendbodyw1.getStdMin(0)[0:len(frontendbodyw1.t)-2], color=[0,0,0.5], linestyle='-.', lw=2.4)
values=frontendreferencestim300.getAxis(0)
plt.plot(frontendreferencestim300.t, values,
        marker='D', label="Ground Truth data", color=[0,0.5,0.5], alpha=0.5, lw=5)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/normalgrass_velocity_model_different_weight.png')


