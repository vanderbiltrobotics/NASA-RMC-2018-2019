import cv2
import numpy as np

threshold = .5
iterations = 1

# Read image in
elevation_map = cv2.imread('before_filter.PNG', 0)
# cv2.imshow("before", elevation_map)

# Blur image using a Gaussian filter
smooth_map = cv2.GaussianBlur(elevation_map, (5,5), 0)
# cv2.imshow("Gaussian", smooth_map)

# Apply a laplacian filter to find where edges are
laplace_map = cv2.Laplacian(smooth_map, cv2.CV_8U)

# Rescale map with respect to maximum slope
laplace_map = laplace_map / float(np.max(laplace_map))

cv2.imshow("Laplacian", laplace_map)
cv2.waitKey(0)
cv2.destroyAllWindows()

print(laplace_map)

# Apply threshold to image
laplace_map[laplace_map >= threshold] = 1
laplace_map[laplace_map < threshold] = 0

cv2.imshow("Laplacian", laplace_map)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Dilate and erode image
kernel = np.ones((3,3) , np.uint8)
cv2.destroyAllWindows()
laplace_map = cv2.dilate(laplace_map, kernel, iterations=iterations)
cv2.imshow("dilate", laplace_map)
cv2.waitKey(0)
cv2.destroyAllWindows()
laplace_map = cv2.erode(laplace_map, kernel, iterations=iterations)

cv2.imshow("dilate", laplace_map)
cv2.waitKey(0)
cv2.destroyAllWindows()


quit()