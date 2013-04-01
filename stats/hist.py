import matplotlib.pyplot as plt
from argparse import ArgumentParser
import numpy as np
import mahotas


parser = ArgumentParser(description="Heatmap maker")
parser.add_argument('-f', '--file', type=str, nargs=1, required=True, help='Tape input file.')
args = parser.parse_args()
fname = args.file[0]

try:
	dat = np.loadtxt(fname, delimiter=",")
except Exception, e:
	print "Exception: ", e
	exit(-1)

dat = dat[::-1]

flatdat = [item for sublist in dat for item in sublist ]

#print mahotas.thresholding.otsu(flatdat)

mu, sigma = 100, 15

hist, bins = np.histogram(flatdat,bins = 50)
width = 0.7*(bins[1]-bins[0])
center = (bins[:-1]+bins[1:])/2
plt.bar(center, hist, align = 'center', width = width)
plt.show()