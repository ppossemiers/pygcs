import numpy as np
import cv2

def find_marker(image):
	# convert the image to grayscale, blur it, and detect edges
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (5, 5), 0)
	edged = cv2.Canny(gray, 35, 125)

	# find the contours in the edged image and keep the largest one;
	# we'll assume that this is our piece of paper in the image
	(cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	c = max(cnts, key = cv2.contourArea)

	# compute the bounding box of the of the paper region and return it
	return cv2.minAreaRect(c)

def distance_to_camera(knownWidth, focalLength, perWidth):
	# compute and return the distance from the marker to the camera
	return (knownWidth * focalLength) / perWidth

# initialize the known distance from the camera to the object
KNOWN_DISTANCE = 85.5

# initialize the known object width
KNOWN_WIDTH = 21.5

# load the first image that contains an object that is KNOWN TO BE 85.5 cm
# from our camera, then find the qr code in the image, and initialize
# the focal length
image = cv2.imread('code.jpg')
marker = find_marker(image)
focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH
print focalLength
