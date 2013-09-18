#!/usr/bin/env python

import csv, scipy
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import filter_design as fd
import scipy.signal as sig
from uncertainties import *
from uncertainties import ufloat
from uncertainties import unumpy

#You need to load the data classes in advance_plotting.py script

# Specification for our filter
Wp = 0.512   # Cutoff frequency 
Ws = 0.570   # Stop frequency 
Rp = 0.1     # passband maximum loss (gpass)
As = 60      # stoppand min attenuation (gstop)

Filters = {'ellip' : (), 'cheby2' : (), 'butter' : (), 'cheby1' : (),  'bessel' : ()}

# The bessel max order of 8 for this cutoff, can't use
# iirdesign have to use iirfilter.
Filters['bessel'] = fd.iirfilter(8, Wp, btype='lowpass', ftype='bessel')

#FrontEnd Motion model velocity
frontendbody100Hz = ThreeData()
#frontendbody100Hz.readData('data/normal_spacehall/frontend_poseout_velocity.1154.0.data', cov=True)
frontendbody100Hz.readData('data/multitest_spacehall/frontend_poseout_velocity.0940.0.data', cov=True)
frontendbody100Hz.eigenValues()

#Copy the velocity.
tbody = frontendbody100Hz.t
velbodyx = [frontendbody100Hz.data[i][0] for i in range (len(frontendbody100Hz.data))]


velbodyxfilter = {'ellip' : (), 'cheby2' : (), 'butter' : (), 'cheby1' : (),  'bessel' : ()}
velbodyxfilter['bessel'] = sig.lfilter(Filters['bessel'][0], Filters['bessel'][1], velbodyx)

#Copy the velocity with uncertainties
velbodyVarx = [frontendbody100Hz.cov[i][0][0] for i in range (len(frontendbody100Hz.cov))]

#From the uncertainty valriable
uvelbodyx = unumpy.uarray(velbodyx, sqrt(velbodyVarx))

#Manual implementation of the filter
n=8
b=Filters['bessel'][0]
a=Filters['bessel'][1]
velbodyxbessel = [0]*len(uvelbodyx)
velbodyxbessel[0:n-1] = uvelbodyx[0:n-1]

for i in range(n,len(velbodyx)):
    velbodyxbessel[i] = 1.0/a[0] * (b[0]*uvelbodyx[i] + b[1]*uvelbodyx[i-1] + b[2]*uvelbodyx[i-2]
				    + b[3]*uvelbodyx[i-3] + b[4]*uvelbodyx[i-4] + b[5]*uvelbodyx[i-5]
				    + b[6]*uvelbodyx[i-6] + b[7]*uvelbodyx[i-7] + b[8]*uvelbodyx[i-8]
				    - a[1]*velbodyxbessel[i-1] - a[2]*velbodyxbessel[i-2]
				    - a[3]*velbodyxbessel[i-3] - a[4]*velbodyxbessel[i-4]
				    - a[5]*velbodyxbessel[i-5] - a[6]*velbodyxbessel[i-6]
				    - a[7]*velbodyxbessel[i-7] - a[8]*velbodyxbessel[i-8])

#Storing the result in a ThreeData object
ufrontendbody100Hz = ThreeData()
ufrontendbody100Hz.t = tbody
ufrontendbody100Hz.data = []
for i in range(0,len(ufrontendbody100Hz.t)):
	ufrontendbody100Hz.data.append(np.array([unumpy.nominal_values(velbodyxbessel[i]), 0.00, 0.00]))
ufrontendbody100Hz.var=[]
for i in range(0,len(ufrontendbody100Hz.t)):
	ufrontendbody100Hz.var.append(np.array([pow(unumpy.std_devs(velbodyxbessel[i]),2), 0.00, 0.00]))

frontendbody100Hz.plot_axis(1, 0,True, 1, True, [0,0,1])
ufrontendbody100Hz.plot_axis(1, 0,True, 1, True, [0,1,1])

#Plotting
plt.figure(5)
plt.plot(tbody,velbodyx, '-o', label="X model velocity estimation")
plt.plot(tbody,velbodyxfilter['bessel'], '-o', label="X model velocity bessel filter")
plt.plot(tbody,velbodyxbessel, '-o', label="X model velocity bessel filter(raw)")
legend()
grid(True)

#Tests
x= unumpy.uarray([2.0, 4.0], [1.0, 1.0])
y=[]
y.append(x[0])
y.append(1.0 * x[1] - 1.00 * y[0])

z1= unumpy.uarray([4.0], [1.0])
z2= unumpy.uarray([2.0], [1.0])

upositionx=[]
upositionx.append(0.00)
for i in range(1,len(tbody)):
	upositionx.append(upositionx[i-1] + ((0.01) * velbodyxbessel[i]))


covx=[]
covx.append(0.00)
for i in range(1,len(tbody)):
	covx.append(covx[i-1] + ((0.01) * (0.01) * pow(unumpy.std_devs(uvelbodyx[i]),2)))

cov2x=[]
for i in range(0,len(tbody)):
	cov2x.append(((0.01) * (0.01) * frontendbody100Hz.var[i][0]))
