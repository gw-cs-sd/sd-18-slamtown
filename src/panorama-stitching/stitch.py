# USAGE
# python stitch.py --11 images/lab_01.png --12 images/lab_12.png ... --24 images/lab_24

from panorama import Stitcher
import argparse
import imutils
import cv2

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()

ap.add_argument("--1", required=True,
	help="path to the first image, first row")
ap.add_argument("--2", required=True,
	help="path to the second image, first row")
ap.add_argument("--3", required=True,
 	help="path to the first image, second row")
ap.add_argument("--4", required=True,
	help="path to the second image, second row")
ap.add_argument("--5", required=True,
	help="path to the first image, second row")
ap.add_argument("--6", required=True,
	help="path to the second image, second row")
ap.add_argument("--7", required=True,
	help="path to the third image, second row")
ap.add_argument("--8", required=True,
	help="path to the fourth image, second row")

args = vars(ap.parse_args())

color1 = cv2.imread(args["1"])
color2 = cv2.imread(args["2"])
color3 = cv2.imread(args["3"])
color4 = cv2.imread(args["4"])
thermal1 = cv2.imread(args["5"])
thermal2 = cv2.imread(args["6"])
thermal3 = cv2.imread(args["7"])
thermal4 = cv2.imread(args["8"])

stitcher = Stitcher()
stitcher.panorama(color1, color2, color3, color4, thermal1, thermal2, thermal3, thermal4)
# image2_1 = cv2.imread(args["21"])
# image2_2 = cv2.imread(args["22"])
# image2_3 = cv2.imread(args["23"])
# image2_4 = cv2.imread(args["24"])

# image1_1 = imutils.resize(image1_1, width=400)
# image1_2 = imutils.resize(image1_2, width=400)
# image1_3 = imutils.resize(image1_3, width=400)
# image1_4 = imutils.resize(image1_4, width=400)
# image2_1 = imutils.resize(image2_1, width=400)
# image2_2 = imutils.resize(image2_2, width=400)
# image2_3 = imutils.resize(image2_3, width=400)
# image2_4 = imutils.resize(image2_4, width=400)

# stitcher = Stitcher()
# stitcher.stitch_main("C://panorama-stitching/images")
# #First, stitch each half of the top row, 2 frames each
# (result1_12, vis1) = stitcher.stitch([image1_1, image1_2], showMatches=True, down=True)
# (result1_34, vis2) = stitcher.stitch([image1_3, image1_4], showMatches=True, left=True)
# #Next, take those results and stitch them together
# (result1, vis3) = stitcher.stitch([result1_12, result1_34], showMatches= True, right=True) 

# #Do the same for the bottom row
# (result2_12, vis4) = stitcher.stitch([image2_1, image2_2], showMatches=True, right=True)
# (result2_34, vis5) = stitcher.stitch([image2_3, image2_4], showMatches=True, left=True)
# (result2, vis6) = stitcher.stitch([result2_12, result2_34], showMatches=True, right=True)

# #Finally, do a downward stitch of the top and bottom row
# (result3, vis7) = stitcher.stitch([result1, result2], showMatches=True, down=True)


# # show the images
# #cv2.imshow("Image 1A", image1A)
# #cv2.imshow("Image 1B", image1B)
# #cv2.imshow("Image 1C", image1C)
# #cv2.imshow("Image 1D", image1D)
# ##cv2.imshow("Keypoint Matches 1_3 and 1_4", vis2)
# #cv2.imshow("Keypoint Matches 1_12 and 1_34", vis3)
# cv2.imshow("Result1_12", result1_12)
# #cv2.imshow("Result1_34", result1_34)
# #cv2.imshow("Result1", result1)
# cv2.imshow("Keypoint Matches 1_1 and 2_1", vis1)
# cv2.imshow("Result12_1", result12_1)
cv2.waitKey(0)