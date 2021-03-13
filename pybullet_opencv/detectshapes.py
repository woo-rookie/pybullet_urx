from pybullet_opencv.shapedetector import ShapeDetector
import numpy as np
import imutils
import cv2

class DetectShape:
	def __init__(self,inputImage):
		# load the image and resize it to a smaller factor so that
		# the shapes can be approximated better
		hsv = cv2.cvtColor(inputImage, cv2.COLOR_BGR2HSV)
		lowergreen = np.array([35,43,46],dtype=np.uint8)
		uppergreen = np.array([99,255,255],dtype=np.uint8)
		maskgreen = cv2.inRange(hsv,lowergreen,uppergreen)
		maskgreen = cv2.bitwise_not(maskgreen)
		self.image = cv2.bitwise_and(inputImage, inputImage, mask=maskgreen)
		# self.image = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
		resized = imutils.resize(self.image, width=300)
		self.ratio = self.image.shape[0] / float(resized.shape[0])
		# convert the resized image to grayscale, blur it slightly,
		# and threshold it
		gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(gray, (5, 5), 0)
		thresh = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)[1]
		cv2.imshow("captured image", inputImage)
		# find contours in the thresholded image and initialize the
		# shape detector
		self.cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
								cv2.CHAIN_APPROX_SIMPLE)
		self.cnts = imutils.grab_contours(self.cnts)
		self.sd = ShapeDetector()
		pass
	def detectShape(self):
		# loop over the contours
		for c in self.cnts:
			# compute the center of the contour, then detect the name of the
			# shape using only the contour
			M = cv2.moments(c)
			cX = int((M["m10"] / M["m00"]) * self.ratio)
			cY = int((M["m01"] / M["m00"]) * self.ratio)
			shape = self.sd.detect(c)

			# multiply the contour (x, y)-coordinates by the resize ratio,
			# then draw the contours, the name of the shape on the image
			# and the center of the contour
			c = c.astype("float")
			c *= self.ratio
			c = c.astype("int")
			cv2.drawContours(self.image, [c], -1, (0, 255, 0), 2)
			cv2.circle(self.image, (cX, cY), 4, (0, 0, 0), -1)
			cv2.putText(self.image, shape, (cX - 20, cY + 30),
						cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
			cv2.putText(self.image, "center", (cX - 20, cY - 20),
						cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


			# show the output image
			cv2.imshow("detected image", self.image)
