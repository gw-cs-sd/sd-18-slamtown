This folder contains files pertaining to image stitching for color and thermal images in our Distributed Climate Control project
The stitching concepts used in these files come from an OpenCV artice on Panorama Stitching by Adrian Rosebrock
Link: https://www.pyimagesearch.com/2016/01/11/opencv-panorama-stitching/

In order to use the panorama.py library, you will need to either use OpenCV 2.4X or OpenCV 3.X with the contrib module. For simplicity, it is easier to do the latter. First, do a pip install opencv-python on your windows machine. Next, do a pip install opencv-contrib-python. The reason we need the contrib module is because every version OpenCV after 3 has gotten rid of its SIFT and SURF implementations, which are utilized for keypoint detection in our code. More information here: https://www.pyimagesearch.com/2015/07/16/where-did-sift-and-surf-go-in-opencv-3/

The stitching works by first creating a homography matrix based on extracted features from both images. Based on an input to the stitch method, one image is chosen as the "base" for the stitch. The base does not change at all, and the other image is warped in order to match up correctly with the base. Features are extracted from both images using the detectAndDescribe method. Using the homography generated from this, the non-base image is warped using the cv2.warpPerspective method, and placed in a new image with double length or height of the originals. The base image is placed on one side depending on the entered orientation of the stitch.

We use a grid of four images in a square for our scenes. Thus we stitching right-ward on both rows, and then apply a downward stitch to the results. This generally comes out with a better result than doing it the other way around. We use the same homography from each stitch on both our color and thermal images, such that they line up correctly. Thus we input both the thermal and color images for each stitch.

