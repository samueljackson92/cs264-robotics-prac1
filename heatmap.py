import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

dat = np.loadtxt('output.csv', delimiter=",")

lum_img = dat[:,:]
imgplot = plt.imshow(lum_img)
imgplot.set_cmap('hot')
#imgplot.set_clim(10,50)
imgplot.set_interpolation('nearest')
plt.show()