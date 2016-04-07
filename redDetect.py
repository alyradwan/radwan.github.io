import numpy as np
import cv2
from PIL import Image


class Crop(object):
    def __init__(self):
        pass

    def four_point_transform(self,image, rect):
        tl = rect[0]
        tr = rect[1]
        br = rect[2]
        bl = rect[3]
        widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        maxWidth = max(int(widthA), int(widthB))
        # compute the height of the new image, which will be the
        # maximum distance between the top-right and bottom-right
        # y-coordinates or the top-left and bottom-left y-coordinates
        heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
        heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
        maxHeight = max(int(heightA), int(heightB))

        # now that we have the dimensions of the new image, construct
        # the set of destination points to obtain a "birds eye view",
        # (i.e. top-down view) of the image, again specifying points
        # in the top-left, top-right, bottom-right, and bottom-left
        # order
        dst = np.array([
            [0, 0],
            [maxWidth - 1, 0],
            [maxWidth - 1, maxHeight - 1],
            [0, maxHeight - 1]], dtype="float32")

        # compute the perspective transform matrix and then apply it
        M = cv2.getPerspectiveTransform(rect, dst)
        warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))

        # return the warped image
        return warped

    def mainCrop(self,s):

        image = cv2.imread(s)
        # s = 'redtest.jpg'
        # define the list of boundaries
        boundaries = [
            ([0, 0, 100], [50, 50, 131])
        ]

        for (lower, upper) in boundaries:
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")
            # find the colors within the specified boundaries and apply
            #    the mask
            im = Image.open(s)
            im = im.convert('RGB')
            w, h = im.size
            mask = cv2.inRange(image, lower, upper)
            red = []
            factor = 10
            for j in range(0, h - 1, factor):
                for i in range(0, w - 1, factor):
                    if mask[j][i] > 0:
                        red.append((i, j))

            # print (red)
            averagered = []
            for pi in range(len(red) - 1):
                if abs(red[pi][0] - red[pi + 1][0]) > factor * 5 or abs(red[pi][1] - red[pi + 1][1]) > factor * 5:
                    print ("change in :", pi)
                    averagered.append((red[pi][0], red[pi][1]))
            averagered.append((red[len(red) - 1][0], red[len(red) - 1][1]))
            a = np.array([
                [averagered[0][0], averagered[0][1]],
                [averagered[1][0], averagered[1][1]],
                [averagered[3][0], averagered[3][1]],
                [averagered[2][0], averagered[2][1]]], np.float32)

            cropped = Crop().four_point_transform(image, a)
            ret, thresh1 = cv2.threshold(cropped, 127, 255, cv2.THRESH_BINARY)
            thresh1 = cv2.resize(thresh1, None, fx=0.33, fy=0.33, interpolation=cv2.INTER_AREA)
            cv2.imwrite('croppedwiththresh.jpg', thresh1)
            return 'croppedwiththresh.jpg'
            # output = cv2.bitwise_and(image, image, mask=mask)
            # cv2.imshow('thresh', thresh1)
            # cv2.waitKey(0)
