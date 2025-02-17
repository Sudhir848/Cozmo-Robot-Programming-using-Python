import cv2
import numpy as np

#TODO: Modify these values for yellow color range. Add thresholds for detecting green also.
yellow_lower = np.array([11, 140, 134])
yellow_upper = np.array([35, 237, 227])
green_lower = np.array([15, 19, 0])
green_upper = np.array([125, 183, 69])


#TODO: Change this function so that it filters the image based on color using the hsv range for each color.
def filter_image(img, hsv_lower_yellow, hsv_upper_yellow, hsv_lower_green, hsv_upper_green):

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Modify mask
    mask_yellow = cv2.inRange(hsv, hsv_lower_yellow, hsv_upper_yellow)
    mask_green = cv2.inRange(hsv, hsv_lower_green, hsv_upper_green)

    # Applying median blurring to the mask
    blurred_mask_yellow = cv2.medianBlur(mask_yellow, 9)
    blurred_mask_green = cv2.medianBlur(mask_green, 9)

    # Displaying the mask to verify it's correct
    # cv2.imshow('Mask', blurred_mask_yellow)
    # cv2.imshow('Mask', blurred_mask_green)
    # hasYellow = np.sum(blurred_mask_yellow)
    # hasGreen = np.sum(blurred_mask_green)
    # if hasYellow > 0:
    #     print('Yellow detected!')
    #     print(f'Amount: {hasYellow}')  
    # if hasGreen > 0:
    #     print('Green detected!')
    #     print(f'Amount: {hasGreen}')   
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    return blurred_mask_yellow, blurred_mask_green


#TODO: Change the parameters to make blob detection more accurate. Hint: You might need to set some parameters to specify features such as color, size, and shape. The features have to be selected based on the application. 
def detect_blob(mask):

    # Set up the SimpleBlobdetector with default parameters with specific values.
    params = cv2.SimpleBlobDetector_Params()

    #ADD CODE HERE
    # Filter by Color
    params.filterByColor = True
    params.blobColor = 255
    
    # Filter by Size
    params.filterByArea = True
    params.minArea = 500
    params.maxArea = 5000

    # Filter by Shape
    params.filterByCircularity = False
    params.filterByConvexity = False

    # builds a blob detector with the given parameters 
    detector = cv2.SimpleBlobDetector_create(params)

    # use the detector to detect blobs.
    keypoints = detector.detect(mask)

    return keypoints


def count_cubes(img):
    mask_yellow, mask_green = filter_image(img, yellow_lower, yellow_upper, green_lower, green_upper)

    keypoints_yellow = detect_blob(mask_yellow)
    num_yellow = len(keypoints_yellow)

    keypoints_green = detect_blob(mask_green)
    num_green = len(keypoints_green)

    #TODO: Modify to return number of detected cubes for both yellow and green (instead of 0)
    return num_yellow, num_green
