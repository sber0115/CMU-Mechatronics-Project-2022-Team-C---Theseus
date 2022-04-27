import cv2 as cv
from os.path import join


def cannyEdgeDetector(_):
    global low_threshold
    global high_threshold
    global aperture
    global gradient
    # reads values from trackbars
    low_threshold = cv.getTrackbarPos('Low Threshold:', 'Feature Map' )
    high_threshold = cv.getTrackbarPos('High Threshold', 'Feature Map')
    aperture = cv.getTrackbarPos('Aperture', 'Feature Map')
    gradient = cv.getTrackbarPos('Gradient on/off', 'Feature Map')
    smooth_img = cv.blur(original, (5, 5))
    # converts inputs to 3, 5, 7 for aperture size
    if aperture == 0:
        aperture = 3
    elif aperture == 1:
        aperture = 5
    elif aperture == 2:
        aperture = 7
    feature_map = cv.Canny(smooth_img, low_threshold, high_threshold, apertureSize=aperture, L2gradient=gradient)
    feature_map = cv.bitwise_not(feature_map)
    cv.imshow('Feature Map', feature_map)
    cv.imwrite(img_path+"-canny.png", feature_map)  # saves image


def main():
    folder_path = "templates"
    files_names = ['cheerios', 'circuit', 'gear', 'professor']
    files_names = ['small_valve_template']

    # creats trackbars
    cv.namedWindow('Feature Map')
    cv.createTrackbar('Low Threshold:', 'Feature Map' , 0, 255, cannyEdgeDetector)
    cv.createTrackbar('High Threshold', 'Feature Map', 0, 255, cannyEdgeDetector)
    cv.createTrackbar('Aperture', 'Feature Map', 0, 2, cannyEdgeDetector)
    cv.createTrackbar('Gradient on/off', 'Feature Map', False, True, cannyEdgeDetector)

    # loops through all images
    for img in files_names:
        global img_path
        global original
        img_path = join(folder_path, img)
        original = cv.imread(img_path + '.jpg', cv.IMREAD_GRAYSCALE)
        cannyEdgeDetector(0)
        cv.waitKey()

        # displays final parameters
        print("Canny Edge Detector Parameters For Image \"{}\":".format(img))
        print("Low Treshold: {}".format(low_threshold))
        print("High Threshold: {}".format(high_threshold))
        print("Aperture Size: {}".format(aperture))
        print("Gradient: {}\n".format(gradient))
        

if __name__ == '__main__':
    main()