#!/usr/bin/env python

import csv, scipy
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

def func(x):
    return (x-3)*(x-5)*(x-7)+85
    
class VeloData:
    time
    delta
    t
    x
    y
    z
    
    def __init__(self):
	self.time = []
	self.delta = []
	self.t=[]
	self.x=[]
	self.y=[]
	self.z=[]
	
    def f(self):
        return 'hello world'
    
    def readData(self,filename):
	
	for row in csv.reader(open(filename, 'rb'), delimiter=' ', quotechar='|'):
	    #print row
	    self.time.append(float(row[0])/1000000)
	    self.x.append(float(row[1]))
	    self.y.append(float(row[2]))
	    self.z.append(float(row[3]))
	    
	time = self.time     
	for i in range(0,len(time)-1):
	    tbody = float(time[i+1]) - float(time[i])
	    self.delta.append(tbody)
	    
	deltabody_t = mean(self.delta)    
	self.t = deltabody_t * r_[0:len(self.time)]


velbody = VeloData()
velbody.readData('data/minitest_spacehall/spacehall1852.puremodel_velo.incre.0.data')

ax = subplot(111)

a, b = 2, 9 # integral area
x = arange(0, 10, 0.01)
y = func(x)
plot(x, y, linewidth=1)

# make the shaded region
ix = arange(a, b, 0.01)
iy = func(ix)
verts = [(a,0)] + list(zip(ix,iy)) + [(b,0)]
poly = Polygon(verts, facecolor='0.8', edgecolor='k')
ax.add_patch(poly)

text(0.5 * (a + b), 30,
     r"$\int_a^b f(x)\mathrm{d}x$", horizontalalignment='center',
     fontsize=20)

axis([0,10, 0, 180])
figtext(0.9, 0.05, 'x')
figtext(0.1, 0.9, 'y')
ax.set_xticks((a,b))
ax.set_xticklabels(('a','b'))
ax.set_yticks([])
show()