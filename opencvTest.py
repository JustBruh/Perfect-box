import cv2
import numpy as np
from matplotlib import pyplot as plt

# Declaring the output graph's size

plt.figure(figsize=(16, 16))

# Convert image to grayscale
img_gs = cv2.imread('opencv_test.jpg', cv2.IMREAD_GRAYSCALE)
cv2.imwrite('gs.jpg', img_gs)

# Apply canny edge detector algorithm on the image to find edges



edges = cv2.Canny(img_gs, 100,200)
print(edges)
cv2.imwrite('edges.jpg', edges)




# Plot the original image against the edges
#plt.subplot(121), plt.imwrite('gs.jpg')
#plt.title('Original Gray Scale Image')
#plt.subplot(122), plt.imwrite('gs.jpg', edges)
#plt.title('Edge Image')