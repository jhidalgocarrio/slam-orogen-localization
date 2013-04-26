#!/usr/bin/python
import csv, scipy
from pylab import *
import numpy as np
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt


class SlipData:
    
    def __init__(self):
	self.time = []
	self.delta = []
	self.t=[]
	self.slipvector=[]
	self.contactpoints=[]
	self.nwheels=[]
	self.cov=[]
	self.var=[]
	
    def f(self):
        return 'hello world'
    
    def readData(self,filename, cov=False):
	
	for row in csv.reader(open(filename, 'rb'), delimiter=' ', quotechar='|'):
	    #print row
	    self.time.append(float(row[0])/1000000)
	    #slip vector for the rover
	    self.slipvector.append(np.array([[float(row[1]), float(row[2]), float(row[3])],
					     [float(row[4]), float(row[5]), float(row[6])],
					     [float(row[7]), float(row[8]), float(row[9])],
					     [float(row[10]), float(row[11]), float(row[12])]]))
	    nrows=float(row[13])
	    ncols=float(row[14])
	    self.nwheels = ncols
	    #contact points 3d coordinates
	    self.contactpoints.append(np.array([[float(row[15]), float(row[16]), float(row[17])],
					     [float(row[18]), float(row[19]), float(row[20])],
					     [float(row[21]), float(row[22]), float(row[23])],
					     [float(row[24]), float(row[25]), float(row[26])]]))
	    
	    if False != cov:
		ncols=float(row[25])
		ncols=float(row[26])
		#matrix = np.array([[float(row[27]), float(row[28]), float(row[29]), row[30]), float(row[31]), float(row[32])
					#float(row[33]), float(row[34]), float(row[35]), row[36]), float(row[37]), float(row[38])],
				    #[float(row[39]), float(row[40]), float(row[29]), row[30]), float(row[31]), float(row[32])
					#float(row[33]), float(row[34]), float(row[35]), row[36]), float(row[37]), float(row[38])],
				    #[float(row[27]), float(row[28]), float(row[29]), row[30]), float(row[31]), float(row[32])
					#float(row[33]), float(row[34]), float(row[35]), row[36]), float(row[37]), float(row[38])]])
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
	    
    def plot_slip_arrows(self, fign=1, posx=[], posy=[], xlim=[1.0,-1.0], ylim=[1.0,-1.0], cov=False, grid=False):	
	    
	fig  = plt.figure(fign)
	ax = fig.add_subplot(111)
	ax.set_xlim(xlim[0], xlim[1])
	ax.set_ylim(ylim[0], ylim[1])
	
	valuesRL=[]
	valuesRR=[]
	valuesFR=[]
	valuesFL=[]
	for i in range(0,len(self.t)-1):
	    valuesRL = np.array([self.contactpoints[i][0][0],
				     self.contactpoints[i][0][1],
				     self.slipvector[i][0][0],
				     self.slipvector[i][0][1]])
				     
	    valuesRR = np.array([self.contactpoints[i][1][0],
				     self.contactpoints[i][1][1],
				     self.slipvector[i][1][0],
				     self.slipvector[i][1][1]])
				     
	    valuesFR = np.array([self.contactpoints[i][2][0],
				     self.contactpoints[i][2][1],
				     self.slipvector[i][2][0],
				     self.slipvector[i][2][1]])
				     
	    valuesFL = np.array([self.contactpoints[i][3][0],
				     self.contactpoints[i][3][1],
				     self.slipvector[i][3][0],
				     self.slipvector[i][3][1]])
	    #RL slip arrow in blue
	    plt.arrow(valuesRL[0], valuesRL[1], 0.001, 0.001, fc="b", ec="b",head_width=0.03, head_length=0.03, fill=True, label="RL wheel slip track")
	    
	    #RR slip arrow in green
	    plt.arrow(valuesRR[0], valuesRR[1], 0.001, 0.001, fc="g", ec="g",head_width=0.03, head_length=0.03, fill=True, label="RR wheel slip track")
	    
	    #FR slip arrow in blue
	    plt.arrow(valuesFR[0], valuesFR[1], 0.001, 0.001, fc="k", ec="k",head_width=0.03, head_length=0.03, fill=True, label="FR wheel slip track")
	    
	    #RR slip arrow in green
	    plt.arrow(valuesFL[0], valuesFL[1], 0.001, 0.001, fc="m", ec="m",head_width=0.03, head_length=0.03, fill=True, label="FL wheel slip track")
	    
	    print len(valuesRR)
	    print valuesRR
	    
	
	ax.plot(posx,posy, '--', linewidth=3.5,color='r', label="Estimated rover body center")
	
	if False != cov:
	    plt.fill_between(self.t, sdmax, sdmin, color=linecolor)
	
	if False != grid:
	    plt.grid(True)

	plt.setp(ax.get_xticklabels(), rotation='horizontal', fontsize=26)
	plt.setp(ax.get_yticklabels(), rotation='horizontal', fontsize=26)
	plt.xlabel("Position X-axis (m)", fontsize=28)
	plt.ylabel("Position Y-axis (m)", fontsize=28)
	ax.legend(prop={'size':25})
	plt.show()
	

subplot(111)
# arrow( x, y, dx, dy, **kwargs )
plt.arrow(0.5, 0.8, 0.1, -0.2, fc="b", ec="b",head_width=0.05, head_length=0.1, fill=True, label="Estimated rover body center")
legend()
plt.show()

#Slip vector
slipinfo = SlipData()
slipinfo.readData('data/normal_spacehall/spacehall1057.slip_info.0.data', cov=False)
slipinfo.plot_slip_arrows(1, posviconx, posvicony, [0.5,5.0], [-3.5, 1.5], False, True)
slipinfo.plot_slip_arrows(4, posposex, posposey, [-2.0,5.0], [-3.5, 1.5], False, True)
slipinfo.plot_slip_arrows(4, posposex_wo, posposey_wo, [0.5,4,0], [-2.5, 1.0], False, True)

#spamReader = csv.reader(open('data/normal_spacehall/spacehall1057.pose_out.1.data', 'rb'), delimiter=' ', quotechar='|')
spamReader = csv.reader(open('data/normal_spacehall/spacehall1057.pose_out.0.data', 'rb'), delimiter=' ', quotechar='|')


timepose=[]
posposex=[]
posposey=[]
posposez=[]
velposex=[]
velposey=[]
velposez=[]

for row in spamReader:
    #print row
    timepose.append(float(row[0])/1000000)
    posposex.append(float(row[1]))
    posposey.append(float(row[2]))
    posposez.append(float(row[3]))
    velposex.append(float(row[4]))
    velposey.append(float(row[5]))
    velposez.append(float(row[6]))
    
deltapose = []
for i in range(0,len(timepose)-1):
    #print timebody[i]
    tpose = float(timepose[i+1]) - float(timepose[i])
    deltapose.append(tpose)
    
deltapose_t = mean(deltapose)    
sample_ratepose = 1/deltapose_t
tpose = deltapose_t * r_[0:len(timepose)]

#Only to copy when it is w/o whell weighting
posposex_wo = posposex
posposey_wo = posposey


spamReader = csv.reader(open('data/normal_spacehall/spacehall1057.vicon_position.0.data', 'rb'), delimiter=' ', quotechar='|')


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

deltavicon = []
for i in range(0,len(timevicon)-1):
    #print time[i]
    tvicon = float(timevicon[i+1]) - float(timevicon[i])
    deltavicon.append(tvicon)
    
deltavicon_t = mean(deltavicon)    
sample_ratevicon = 1/deltavicon_t
tvicon = deltavicon_t * r_[0:len(timevicon)]


#fontsize = 0.2 * 70
#x=5
#y=5

#fig1 = plt.figure(10)

#ax = fig1.add_axes([0, 0, 1, 1], frameon=False, aspect=1.)
#ax.set_xlim(0, 10)
#ax.set_ylim(0, 10)
#ax = fig.add_subplot(111)
#p = mpatches.Circle((x, y), 0.2, fc="w")
#ax.add_patch(p)

#ax.annotate("fancy", (x, y),
                #(x-1.2, y),
                ##xycoords="figure fraction", textcoords="figure fraction",
                #ha="right", va="center",
                #size=fontsize,
                #arrowprops=dict(arrowstyle='fancy',
                                #patchB=p,
                                #shrinkA=5,
                                #shrinkB=5,
                                #fc="w", ec="k",
                                #connectionstyle="arc3,rad=-0.05",
                                #),
                #bbox=dict(boxstyle="square", fc="w"))
                
#plt.draw()
#plt.show()
