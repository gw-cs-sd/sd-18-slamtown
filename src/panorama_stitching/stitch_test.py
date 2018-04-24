# USAGE
# python stitch.py --11 images/lab_01.png --12 images/lab_12.png ... --24 images/lab_24

from panorama import Stitcher
import argparse
import imutils
import cv2

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()

ap.add_argument("--1", required=True,
	help="path to the first color image, first row")
ap.add_argument("--2", required=True,
	help="path to the second color image, first row")
ap.add_argument("--3", required=True,
 	help="path to the first color image, second row")
ap.add_argument("--4", required=True,
	help="path to the second color image, second row")
# ap.add_argument("--5", required=True,
# 	help="path to the first thermal image, second row")
# ap.add_argument("--6", required=True,
# 	help="path to the second thermal image, second row")
# ap.add_argument("--7", required=True,
# 	help="path to the third thermal image, second row")
# ap.add_argument("--8", required=True,
# 	help="path to the fourth thermal image, second row")

args = vars(ap.parse_args())

img1 = cv2.imread(args["1"])
img2 = cv2.imread(args["2"])
img3 = cv2.imread(args["3"])
img4 = cv2.imread(args["4"])
# img5 = cv2.imread(args["5"])
# img6 = cv2.imread(args["6"])
# img7 = cv2.imread(args["7"])
# img8 = cv2.imread(args["8"])

img1 = imutils.resize(img1, width=400)
img2 = imutils.resize(img2, width=400)
img3 = imutils.resize(img3, width=400)
img4 = imutils.resize(img4, width=400)

stitcher = Stitcher()
(resultHOR, result, vis1) = stitcher.stitch([img1, img2, img1, img2], showMatches=True)
cv2.imshow("Matches", vis1)
cv2.imshow("Result", resultHOR)
cv2.imshow("ResultHOMOG", result)
cv2.waitKey(0)
(resultVERT, result2, vis2) = stitcher.stitch([img3, img4, img3, img4], showMatches=True, down=True)
cv2.imshow("Matches", vis2)
cv2.imshow("Result", resultVERT)

#stitcher.panorama(color1, color2, color3, color4, thermal1, thermal2, thermal3, thermal4)
# image2_1 = cv2.imread(args["21"])
# image2_2 = cv2.imread(args["22"])
# image2_3 = cv2.imread(args["23"])
# image2_4 = cv2.imread(args["24"])


# stitcher = Stitcher()
# (result1_12, vis1) = stitcher.stitch([image1_1, image1_2], showMatches=True, down=True)
# (result1_34, vis2) = stitcher.stitch([image1_3, image1_4], showMatches=True, left=True)
# (result1, vis3) = stitcher.stitch([result1_12, result1_34], showMatches= True, right=True) 
# (result2_12, vis4) = stitcher.stitch([image2_1, image2_2], showMatches=True, right=True)
# (result2_34, vis5) = stitcher.stitch([image2_3, image2_4], showMatches=True, left=True)
# (result2, vis6) = stitcher.stitch([result2_12, result2_34], showMatches=True, right=True)
# (result3, vis7) = stitcher.stitch([result1, result2], showMatches=True, down=True)

cv2.waitKey(0)