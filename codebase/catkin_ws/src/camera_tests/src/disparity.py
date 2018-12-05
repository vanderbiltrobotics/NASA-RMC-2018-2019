

# import required packages
import rospy
import numpy as np
import cv2
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Converter:

	def __init__(self):
		self.left_frame = None
		self.right_frame = None
		self.left_sub = rospy.Subscriber("stereo/left/image_rect", Image, self.left_update)
		self.right_sub = rospy.Subscriber("stereo/right/image_rect", Image, self.right_update)
		self.left_received = False
		self.right_received = False
		self.bridge = CvBridge()


	def left_update(self, data):
		self.left_frame = self.bridge.imgmsg_to_cv2(data, "mono8")
		self.left_received = True


	def right_update(self, data):
		self.right_frame = self.bridge.imgmsg_to_cv2(data, "mono8")
		self.right_received = True

	

# Run program
if __name__ == "__main__":

	# create converter
	converter = Converter()

	# initialize node
	rospy.init_node('compute_disparity', anonymous=True)

	# publisher for disparity images
	image_pub = rospy.Publisher("disparity_image", Image, queue_size=1)

	# Set up image viewer
	ax1 = plt.subplot(131)
	ax2 = plt.subplot(132)
	ax3 = plt.subplot(133)
	ax1.set_aspect('equal')
	ax2.set_aspect('equal')
	ax3.set_aspect('equal')
	im1 = ax1.imshow(converter.left_frame)
	im2 = ax2.imshow(converter.right_frame)
	im3 = ax3.imshow(converter.right_frame, cmap='gray')
	plt.ion

	# compute disparity images
	while not rospy.is_shutdown():
		
		# Wait until some messages received 
		if (converter.left_received and converter.right_received):
	
			# Get left and right frame
			imgL = converter.left_frame
			imgR = converter.right_frame

			# Compute stereo image
			stereo = cv2.StereoSGBM_create(numDisparities=64, blockSize=37)
			disparity = stereo.compute(imgL.T, imgR.T)      # divide by 16 to discard fractional bits

			# # Compute histogram, plot 
			# hist, bins = np.histogram(disparity, bins=20)
			# print hist
			# print bins
			# break

			# print "min: " + str(np.amin(disparity)) + "\tmax: " + str(np.amax(disparity))

			# Convert back to color image for image_viewer
			disparity_8_bit = disparity.astype('uint8').T
			disparity_color = cv2.cvtColor(disparity_8_bit, cv2.COLOR_GRAY2BGR)


			# # Convert back to ROS image message and publish
			# image_message = converter.bridge.cv2_to_imgmsg(disparity_color, encoding="bgr8")
			# image_pub.publish(image_message)


			im1.set_data(imgL)
			im2.set_data(imgR)
			im3.set_data(disparity.T)
			plt.pause(0.2)



