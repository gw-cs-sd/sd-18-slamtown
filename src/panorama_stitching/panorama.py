# import the necessary packages
import numpy as np
import imutils
import cv2

#Assuming 2x2 grid of images, take input in order of 4 color images and 4 thermal images
#We stitch together the top and bottom rows, then the results of those 2 together
#Additionally, we apply the same homography from the color to the thermal
#This is explained in more detail below and in the readme

class Stitcher:
	def __init__(self):
		# determine if we are using OpenCV v3.X
		self.isv3 = imutils.is_cv3()

	def panorama(self, color1, color2, color3, color4, thermal1, thermal2, thermal3, thermal4):
		color1=imutils.resize(color1, width=400)
		color2=imutils.resize(color2, width=400)
		color3=imutils.resize(color3, width=400)
		color4=imutils.resize(color4, width=400)
		thermal1=imutils.resize(thermal1, width=400)
		thermal2=imutils.resize(thermal2, width=400)
		thermal3=imutils.resize(thermal3, width=400)
		thermal4=imutils.resize(thermal4, width=400)

		stitcher = Stitcher()
		(ColorPanTop, ThermalPanTop) = stitcher.stitch([color1, color2, thermal1, thermal2], right=True)
		(ColorPanBottom, ThermalPanBottom) = stitcher.stitch([color3, color4, thermal3, thermal4], right=True)
		(ColorPanFull, ThermalPanFull) = stitcher.stitch([ColorPanTop, ColorPanBottom, ThermalPanTop, ThermalPanBottom], down=True)

		return (ColorPanFull, ThermalPanFull)

	def stitch(self, images, ratio=0.75, reprojThresh=4.0,
		showMatches=False, left = False, right = False, down = False):
		# unpack the images, then detect keypoints and extract
		# local invariant descriptors from them
		(colorB, colorA, thermalB, thermalA) = images
		# We only need to extract descriptors from color since we are using the same homoraphy
		(kpsA, featuresA) = self.detectAndDescribe(colorA)
		(kpsB, featuresB) = self.detectAndDescribe(colorB)

		# match features between the  two images
		M = self.matchKeypoints(kpsA, kpsB,
			featuresA, featuresB, ratio, reprojThresh)

		# if the match is None, then there aren't enough matched keypoints to create a panorama
		if M is None:
			return None

		# otherwise, apply a perspective warp to stitch the images together
		(matches, H, status) = M
		# Image on the left side is warped
		if left:
			# warpPerspective's first input is the image to be warped, thus imageB.
			# second is the homography matrix, and then the dimensions of the result
			result = cv2.warpPerspective(colorB, H,
				(colorA.shape[1] + colorB.shape[1], colorA.shape[0]))
			result[0:colorB.shape[0], colorB.shape[1]:(colorB.shape[1]*2)] = colorA
			# Now apply same homography to stitch thermal
			resultThermal = cv2.warpPerspective(thermalB, H,
				thermalA.shape[1] + thermalB.shape[1], thermalA.shape[0])
			resultThermal[0:thermalB.shape[0], thermalB.shape[1]:(thermalB.shape[1]*2)] = thermalA
		elif down:
			result = cv2.warpPerspective(colorA, H,
				(colorA.shape[1], colorA.shape[0] + colorB.shape[0]))
			result[0:colorB.shape[0], 0:colorB.shape[1]] = colorB

			resultThermal = cv2.warpPerspective(thermalA, H,
				(thermalA.shape[1], thermalA.shape[0] + thermalB.shape[0]))
			resultThermal[0:thermalB.shape[0], 0:thermalB.shape[1]] = thermalB
		# Assume right if neither above is satisfied (primarily used)
		else:
			result = cv2.warpPerspective(colorA, H,
				(colorA.shape[1] + colorB.shape[1], colorA.shape[0]))
			result[0:colorB.shape[0], 0:colorB.shape[1]] = colorB

			resultThermal = cv2.warpPerspective(thermalA, H, 
				(thermalA.shape[1] + thermalB.shape[1], thermalA.shape[0]))
			resultThermal[0:thermalB.shape[0], 0:thermalB.shape[1]] = thermalB
		# Image on the lower side is warped
		
		# Mostly for visualization/sanity check, visualize keypoint matches
		if showMatches:
			vis = self.drawMatches(colorB, colorA, kpsB, kpsA, matches,
				status)

			# Return a tuple of the stitched image and the visualization
			return (result, resultThermal, vis)

		# Otherwise just return the stitched images
		return (result, resultThermal)

	def detectAndDescribe(self, image):
		# Convert the image to grayscale
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		# Assuming that we are using OpenCV 3
		# detect and extract features from the image
		descriptor = cv2.xfeatures2d.SIFT_create()
		(kps, features) = descriptor.detectAndCompute(image, None)

		# convert the keypoints from KeyPoint objects to NumPy arrays
		kps = np.float32([kp.pt for kp in kps])

		# return a tuple of keypoints and features
		return (kps, features)

	def matchKeypoints(self, kpsA, kpsB, featuresA, featuresB,
		ratio, reprojThresh):
		# compute the raw matches and initialize the list of actual
		# matches
		matcher = cv2.DescriptorMatcher_create("BruteForce")
		rawMatches = matcher.knnMatch(featuresA, featuresB, 2)
		matches = []

		# loop over the raw matches
		for m in rawMatches:
			# ensure the distance is within a certain ratio of each
			# other (i.e. Lowe's ratio test)
			if len(m) == 2 and m[0].distance < m[1].distance * ratio:
				matches.append((m[0].trainIdx, m[0].queryIdx))

		# computing a homography requires at least 4 matches
		if len(matches) > 4:
			# construct the two sets of points
			ptsA = np.float32([kpsA[i] for (_, i) in matches])
			ptsB = np.float32([kpsB[i] for (i, _) in matches])

			# compute the homography between the two sets of points
			(H, status) = cv2.findHomography(ptsA, ptsB, cv2.RANSAC,
				reprojThresh)

			# return the matches along with the homograpy matrix
			# and status of each matched point
			return (matches, H, status)

		# otherwise, no homograpy could be computed
		return None

	def drawMatches(self, imageA, imageB, kpsA, kpsB, matches, status):
		# initialize the output visualization image
		(hA, wA) = imageA.shape[:2]
		(hB, wB) = imageB.shape[:2]
		vis = np.zeros((max(hA, hB), wA + wB, 3), dtype="uint8")
		vis[0:hA, 0:wA] = imageA
		vis[0:hB, wA:] = imageB

		# loop over the matches
		for ((trainIdx, queryIdx), s) in zip(matches, status):
			# only process the match if the keypoint was successfully
			# matched
			if s == 1:
				# draw the match
				ptA = (int(kpsA[queryIdx][0]), int(kpsA[queryIdx][1]))
				ptB = (int(kpsB[trainIdx][0]) + wA, int(kpsB[trainIdx][1]))
				cv2.line(vis, ptA, ptB, (0, 255, 0), 1)

		# return the visualization
		return vis
