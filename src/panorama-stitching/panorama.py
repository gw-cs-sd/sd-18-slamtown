# import the necessary packages
import numpy as np
import imutils
import cv2

#Assuming 2x4 grid of images, scan given directory for appropriate images. Images are organized with a date and a row_column identifier. For example, 2018_03_12_1_3 for
#the 3rd image in the first row. Perform left stitches on the first and second images of both rows, and right stitches on the third and fourth images as well. This is explained 
#further in the readme. Finally, stitch together  bottom and stop rows using downward stitching.

class Stitcher:
	def __init__(self):
		# determine if we are using OpenCV v3.X
		self.isv3 = imutils.is_cv3()

	def panorama(self, image1, image2, image3, image4):
		image1=imutils.resize(image1, width=400)
		image2=imutils.resize(image2, width=400)
		image3=imutils.resize(image3, width=400)
		image4=imutils.resize(image4, width=400)

		stitcher = Stitcher()
		(PanTop) = stitcher.stitch([image1, image2], right=True)
		(PanBottom) = stitcher.stitch([image3, image4], right=True)
		(PanFull) = stitcher.stitch([PanTop, PanBottom], down=True)

		cv2.imshow("PanTop", PanTop)
		cv2.imshow("PanBottom", PanBottom)
		cv2.imshow("PanFull", PanFull)

	def stitch(self, images, ratio=0.75, reprojThresh=4.0,
		showMatches=False, left = False, right = False, down = False):
		# unpack the images, then detect keypoints and extract
		# local invariant descriptors from them
		(imageB, imageA) = images
		(kpsA, featuresA) = self.detectAndDescribe(imageA)
		(kpsB, featuresB) = self.detectAndDescribe(imageB)

		# match features between the  two images
		M = self.matchKeypoints(kpsA, kpsB,
			featuresA, featuresB, ratio, reprojThresh)

		# if the match is None, then there aren't enough matched keypoints to create a panorama
		if M is None:
			return None

		# otherwise, apply a perspective warp to stitch the images together
		(matches, H, status) = M
		#left stitching, meaning that the image on the left side is warped
		if left:
			#warpPerspective's first input is the image to be warped, thus imageB.
			#second is the homography matrix, and then the dimensions of the result
			result = cv2.warpPerspective(imageB, H,
				(imageA.shape[1] + imageB.shape[1], imageA.shape[0]))
			#imageA is the base image, as in the one that is not altered
			#Thus we set the second half of the image to just be the original imageA
			#With the warped perspective it should match up close enough for a panorama
			result[0:imageB.shape[0], imageB.shape[1]:(imageB.shape[1]*2)] = imageA
		#right stitching, meaning that the image on the right side is warped
		elif right:
			result = cv2.warpPerspective(imageA, H,
				(imageA.shape[1] + imageB.shape[1], imageA.shape[0]))
			result[0:imageB.shape[0], 0:imageB.shape[1]] = imageB
			#down stitching, meaning that the lower image is warped
		elif down:
			result = cv2.warpPerspective(imageA, H,
				(imageA.shape[1], imageA.shape[0] + imageB.shape[0]))
			result[0:imageB.shape[0], 0:imageB.shape[1]] = imageB

		# mostly for visualization/sanity check, visualize keypoint matches
		if showMatches:
			vis = self.drawMatches(imageB, imageA, kpsB, kpsA, matches,
				status)

			# return a tuple of the stitched image and the visualization
			return (result, vis)

		# return the stitched image
		return result

	def detectAndDescribe(self, image):
		# convert the image to grayscale
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		# check to see if we are using OpenCV 3.X
		if self.isv3:
			# detect and extract features from the image
			descriptor = cv2.xfeatures2d.SIFT_create()
			(kps, features) = descriptor.detectAndCompute(image, None)

		# otherwise, we are using OpenCV 2.4.X
		else:
			# detect keypoints in the image
			detector = cv2.FeatureDetector_create("SIFT")
			kps = detector.detect(gray)

			# extract features from the image
			extractor = cv2.DescriptorExtractor_create("SIFT")
			(kps, features) = extractor.compute(gray, kps)

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