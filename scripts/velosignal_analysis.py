#!/usr/bin/env python

import csv, scipy, infpy, numpy, pylab
import scipy.fftpack
from pylab import *
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from scipy.signal import filter_design as fd
import scipy.signal as sig

import infpy.gp.kernel_short_names as kernels

# a function to save plots
def save_fig(prefix):
    "Save current figure in extended postscript and PNG formats."
    pylab.savefig('%s.png' % prefix, format='PNG')
    
def mfreqz(b,a=1):
    import scipy.signal as signal
    w,h = signal.freqz(b,a)
    h_dB = 20 * log10 (abs(h))
    subplot(211)
    plot(w/max(w),h_dB)
    ylim(-150, 5)
    ylabel('Magnitude (db)')
    xlabel(r'Normalized Frequency (x$\pi$rad/sample)')
    title(r'Frequency response')
    subplot(212)
    h_Phase = unwrap(arctan2(imag(h),real(h)))
    plot(w/max(w),h_Phase)
    ylabel('Phase (radians)')
    xlabel(r'Normalized Frequency (x$\pi$rad/sample)')
    title(r'Phase response')
    subplots_adjust(hspace=0.5)
    show()

def impz(b,a=1):
    import scipy.signal as signal
    impulse = repeat(0.,50); impulse[0] =1.
    x = arange(0,50)
    response = signal.lfilter(b,a,impulse)
    subplot(211)
    stem(x, response)
    ylabel('Amplitude')
    xlabel(r'n (samples)')
    title(r'Impulse response')
    subplot(212)
    step = cumsum(response)
    stem(x, step)
    ylabel('Amplitude')
    xlabel(r'n (samples)')
    title(r'Step response')
    subplots_adjust(hspace=0.5)
    show()

#Load model velocity
spamReader = csv.reader(open('data/normal_spacehall/spacehall1140.puremodel_velo.slip_gp.2.data', 'rb'), delimiter=' ', quotechar='|')
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

plt.figure(4)
pylab.plot(tbody,velbodyx, '-o', label="X model velocity estimation")
pylab.plot(terror,velerrorx, '-o', label="X velocity error")
pylab.grid()
xlabel("Time(s)")
ylabel("Velocity(m/s)")
title("Asguard Body Velocity - Slip Path")
legend()
plt.show()

#Perform the FFT
fftvelbodyx = fftpack.fft(velbodyx, len(velbodyx))


f=arange(-3343,3342,1)
plt.figure(4)
grid(True)
plot(f,abs(concatenate((fftvelbodyx[3342:],fftvelbodyx[:3342]))))


freqs = fftfreq(tbody.size, deltabody_t)

pylab.subplot(211)
pylab.plot(tbody, velbodyx)
pylab.subplot(212)
pylab.plot(freqs,20*scipy.log10(abs(fftvelbodyx)))
pylab.show()

# Specification for our filter
Wp = 0.470   # Cutoff frequency 
Ws = 0.412   # Stop frequency 
Rp = 0.1     # passband maximum loss (gpass)
As = 60      # stoppand min attenuation (gstop)

Filters = {'ellip' : (), 'cheby2' : (), 'butter' : (), 'cheby1' : (),  'bessel' : ()}

# The ellip and cheby2 filter design
Filters['ellip'] = fd.iirdesign(Wp, Ws, Rp, As, ftype='ellip')
Filters['cheby2'] = fd.iirdesign(Wp, Ws, Rp, As, ftype='cheby2')

# The butter and cheby1 need less constraint spec
Rpl = Rp*10; Asl = As/4.
Filters['butter'] = fd.iirdesign(Wp, Ws, Rp, As, ftype='butter')
Filters['cheby1'] = fd.iirdesign(Wp, Ws, Rp, As, ftype='cheby1')

# The bessel max order of 8 for this cutoff, can't use
# iirdesign have to use iirfilter.
Filters['bessel'] = fd.iirfilter(8, Wp, btype='lowpass', ftype='bessel')


#mfreqz(Filters['ellip'][0], Filters['ellip'][1])


velbodyxfilter = {'ellip' : (), 'cheby2' : (), 'butter' : (), 'cheby1' : (),  'bessel' : ()}
velbodyxfilter['ellip'] = sig.lfilter(Filters['ellip'][0], Filters['ellip'][1], velbodyx)
velbodyxfilter['cheby2'] = sig.lfilter(Filters['cheby2'][0], Filters['cheby2'][1], velbodyx)
velbodyxfilter['butter'] = sig.lfilter(Filters['butter'][0], Filters['butter'][1], velbodyx)
velbodyxfilter['cheby1'] = sig.lfilter(Filters['cheby1'][0], Filters['cheby1'][1], velbodyx)
velbodyxfilter['bessel'] = sig.lfilter(Filters['bessel'][0], Filters['bessel'][1], velbodyx)




#Plots
plt.figure(4)
pylab.plot(tbody,velbodyx, '-o', label="X model velocity estimation")
pylab.plot(tbody,velbodyxfilter['ellip'], '-o', label="X model velocity ellip filter")
pylab.plot(tbody,velbodyxfilter['cheby2'], '-o', label="X model velocity cheby2 filter")
pylab.plot(tbody,velbodyxfilter['butter'], '-o', label="X model velocity butter filter")
pylab.plot(tbody,velbodyxfilter['cheby1'], '-o', label="X model velocity cheby1 filter")
pylab.plot(tbody,velbodyxfilter['bessel'], '-o', label="X model velocity bessel filter")
legend()
grid(True)


#Raw implementation of the Bessel filter
n=8
b=Filters['bessel'][0]
a=Filters['bessel'][1]
velbodyxbessel = [0]*len(velbodyx)
velbodyxbessel[0:n-1] = numpy.array(velbodyx[0:n-1])

for i in range(n,len(velbodyx)):
    velbodyxbessel[i] = 1.0/a[0] * (b[0]*velbodyx[i] + b[1]*velbodyx[i-1] + b[2]*velbodyx[i-2]
				    + b[3]*velbodyx[i-3] + b[4]*velbodyx[i-4] + b[5]*velbodyx[i-5]
				    + b[6]*velbodyx[i-6] + b[7]*velbodyx[i-7] + b[8]*velbodyx[i-8]
				    - a[1]*velbodyxbessel[i-1] - a[2]*velbodyxbessel[i-2]
				    - a[3]*velbodyxbessel[i-3] - a[4]*velbodyxbessel[i-4]
				    - a[5]*velbodyxbessel[i-5] - a[6]*velbodyxbessel[i-6]
				    - a[7]*velbodyxbessel[i-7] - a[8]*velbodyxbessel[i-8])

plt.figure(5)
pylab.plot(tbody,velbodyx, '-o', label="X model velocity estimation")
pylab.plot(tbody,velbodyxfilter['bessel'], '-o', label="X model velocity bessel filter")
pylab.plot(tbody,velbodyxbessel, '-o', label="X model velocity bessel filter(raw)")
legend()
grid(True)


#Specgram
#!/usr/bin/env python
from pylab import *

NFFT = 1024       # the length of the windowing segments
Fs = int(1.0/deltabody_t)  # the sampling frequency

# Pxx is the segments x freqs array of instantaneous power, freqs is
# the frequency vector, bins are the centers of the time bins in which
# the power is computed, and im is the matplotlib.image.AxesImage
# instance

ax1 = subplot(211)
plot(tbody, velbodyx)
subplot(212, sharex=ax1)
Pxx, freqs, bins, im = specgram(velbodyx, NFFT=NFFT, Fs=Fs, noverlap=900,
                                cmap=cm.gist_heat)
show()

