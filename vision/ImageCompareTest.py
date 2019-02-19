# This script compares a list of images and looks for any pixel whose standard
# deviation over the set of pixels at that point is greater than 0, then sets it
# to be white (255). It is for grayscale images, and runs slowly over large
# images. 

import cv2
import numpy as np

imgArr = []

# NOTE: I did not upload the test images I used to the GitHub, because the
# specific images used does not matter so long as they are grayscale.
# Anyone can make images with paint (or just ask Daniel for the images).

# Only ever have one set uncommented at once
#First set of test images - small
#img1 = cv2.imread ("TestImage1.png")
#img2 = cv2.imread ("TestImage2.png")
#img3 = cv2.imread ("TestImage3.png")
#img4 = cv2.imread ("TestImage4.png")
#img5 = cv2.imread ("TestImage5.png")

#Second set of test images - medium
img1 = cv2.imread ("TI6.png")
img2 = cv2.imread ("TI7.png")
img3 = cv2.imread ("TI8.png")
img4 = cv2.imread ("TI9.png")
img5 = cv2.imread ("TI10.png")

#Third set of test images - large
#img1 = cv2.imread ("TI11.png")
#img2 = cv2.imread ("TI12.png")
#img3 = cv2.imread ("TI13.png")
#img4 = cv2.imread ("TI14.png")
#img5 = cv2.imread ("TI15.png")

#Just in case the images aren't actually grayscale
#gray1 = cv2(img1, cv2.COLOR_BGR2GRAY)
#gray2 = cv2(img2, cv2.COLOR_BGR2GRAY)
#gray3 = cv2(img3, cv2.COLOR_BGR2GRAY)
#gray4 = cv2(img4, cv2.COLOR_BGR2GRAY)
#gray5 = cv2(img5, cv2.COLOR_BGR2GRAY)

#imgArr.append(gray1)
#imgArr.append(gray2)
#imgArr.append(gray3)
#imgArr.append(gray4)
#imgArr.append(gray5)


imgArr.append(img1)
imgArr.append(img2)
imgArr.append(img3)
imgArr.append(img4)
imgArr.append(img5)

numImgs = 5                     # numImgs is the number of images used to test
pixArr = [None] * numImgs

leadImg = imgArr[0]                     #Uses img1 as a basis for iterating
height, width = leadImg.shape[:2]
for i in range(0, height):              #Iterates through entire image
    for j in range(0,width):
        for k in range (0,numImgs):
            pixArr[k] = imgArr[k][i,j]  
        if np.std(pixArr) > 0:
            leadImg[i,j] = 255

# To reduce clutter, you can comment out all the cv2.imshow lines besides
# 'Adjusted.'
cv2.imshow('Img1', imgArr[0])    # Possible concern? img1 is adjusted because
cv2.imshow('Img2', imgArr[1])    # leadImg also accesses it
cv2.imshow('Img3', imgArr[2])
cv2.imshow('Img4', imgArr[3])
cv2.imshow('Img5', imgArr[4])
cv2.imshow('Adjusted', leadImg)
cv2.waitKey(0)
cv2.destroyAllWindows()
