#!/usr/bin/env python

##############################################
# Heatmap.py
# Create heatmaps from a csv file using pylot
# Author: 	Samuel Jackson (slj11@aber.ac.uk)
# Date:		26/2/13
##############################################

from argparse import ArgumentParser
import matplotlib.pyplot as plt
import numpy as np
import math

def createHeatmap(img, lower_lim=None, upper_lim=None):
	plt.xlabel('Cell X (60cm per square)')
	plt.ylabel('Cell Y (60cm per square)')

	imgplot = plt.imshow(img, origin='lower')
	imgplot.set_cmap('hot')
	imgplot.set_clim(lower_lim, upper_lim) 
	imgplot.set_interpolation('nearest')

	return hm, imgplot

if __name__ == "__main__":
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

	def average_list(listdat):
		return float(sum(listdat)) / float(len(listdat))

	def weight(listdat):
		maxdat = max(listdat)
		weights_list = [(float(maxdat - item)/ float(maxdat)) for item in listdat]
		return average_list(weights_list)/2

	flatdat = [item for sublist in dat for item in sublist if item > 0 ]
	average = average_list(flatdat)
	threshold = math.ceil(math.floor(average) * weight(flatdat)) 

	print "\nSTATS---------------------------------"
	print "Average Density: %0.3f" % average
	print "Suggested Lower Threshold:  %0.3f" % 5.6

	img = dat[:,:]

	hm = plt.figure(figsize=(12,5))
	hm.suptitle('Heat Map of Occupancy Grid', fontsize=14, fontweight='bold')

	plt.subplot(121)
	plt.title('Without Threshold')
	hm1, imgplot1 = createHeatmap(img)
	plt.subplot(122)
	plt.title('With Lower Threshold : ' + str(34))
	hm2, imgplot2 = createHeatmap(img, 34, None)

	plt.colorbar()
	plt.show()