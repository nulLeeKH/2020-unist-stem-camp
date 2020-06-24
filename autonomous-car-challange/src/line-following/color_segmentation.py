"""
MIT License

Copyright (c) 2020 Lee Kyung-ha <i_am@nulleekh.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""


import cv2
#import imutils
import numpy as np
import pdb

def cd_color_segmentation(img, show_image=False):
    """
	Implement the cone detection using color segmentation algorithm
	    Input:
	    img: np.3darray; the input image with a cone to be detected
	Return:
	    bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
		    (x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
    """
    # convert from rgb to hsv color space (it might be BGR)
    new_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    new_img = new_img[40:, :]
    # new_img = new_img[220:260, :]

    # define lower and upper bound of image values
    # TO DO!
    low_range  = np.array( [-50, 70, 250] )
    high_range = np.array( [50, 245, 255] )

    # create mask for image with overlapping values
    mask = cv2.inRange(new_img, low_range, high_range)

    # filter the image with bitwise and
    filtered = cv2.bitwise_and(new_img, new_img, mask=mask)

    # find the contours in the image
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    x1, y1, x2, y2 = 0, 0, 0, 0
    if len(contours) != 0:
	# find contour with max area, which is most likely the cone
        # Solution note: max uses an anonymous function in this case, we can also use a loop...
        contours_max = max(contours, key = cv2.contourArea)

	# Find bounding box coordinates
        x1, y1, x2, y2 = cv2.boundingRect(contours_max)

	# Draw the bounding rectangle
        cv2.rectangle(img, (x1, y1), (x1 + x2, y1 + y2), (0, 255, 0), 2)

    if show_image:
        cv2.imshow("Color segmentation", img)
        key = cv2.waitKey()
        if key == 'q':
            cv2.destroyAllWindows()

    # Return bounding box
    return ((x1, y1), (x1 + x2, y1 + y2))
