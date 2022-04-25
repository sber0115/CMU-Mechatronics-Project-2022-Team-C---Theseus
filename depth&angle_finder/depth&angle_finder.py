import cv2
import numpy as np


def valvesAngleFinder(frame_gau_blur, hsv, frame, color):
    # creates the range of green
    if color == 'green':
        lower_hsv = np.array([30, 52, 72])
        higher_hsv = np.array([80, 255, 255])
        upper = 2.5
        lower = 2.1
        upper_size = 8000
        lower_size = 1000
    else:
        sensitivity = 50
        lower_hsv = np.array([0, 0, 255-sensitivity])
        higher_hsv = np.array([255, sensitivity, 255])
        upper = 10/7+0.1
        lower = 10/7-0.1
        upper_size = 9000
        lower_size = 1000

    # finds blue regions and extract eblue region dges
    hsv_range = cv2.inRange(hsv, lower_hsv, higher_hsv)
    res_hsv = cv2.bitwise_and(frame_gau_blur,frame_gau_blur, mask=hsv_range)
    hsv_s_gray = cv2.cvtColor(res_hsv, cv2.COLOR_BGR2GRAY)

    _, hsv_s_gray = cv2.threshold(hsv_s_gray, 8, 255, cv2.THRESH_BINARY)
    cnts = cv2.findContours(hsv_s_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # Use index [-2] to be compatible to OpenCV 3 and 4

    for c in cnts:
        rect = cv2.minAreaRect(c)
        (x,y),(w,h), a = rect
        if (lower < w/(h+0.0001) < upper or lower < h/(w+0.0001) < upper) and upper_size>w*h>lower_size: # filter out incorrect rectangle
            box = cv2.boxPoints(rect)
            box = np.int0(box) #turn into ints
            cv2.drawContours(frame, [box], 0, (0,0,255), 4)
            angle = round(a, 2)
            if h<w:
                ori = 'horizontal'
                #if angle < 90.00:
                #   angle += 90.00
            else:
                ori = 'vertical'
            print('Angle:', angle)
            print('Orientation:', ori)
    #cv2.imshow('Circular Valve', frame)
    #cv2.imshow('blue elements', hsv_s_gray)


def valvesFinder(coef, pow, radius, frame, type):
    # select marker type
    if type == 'small valve':
        color = 'green'
    else:
        color = 'white'

    # image pre-processing
    frame_gau_blur = cv2.GaussianBlur(frame, (3, 3), 0)
    hsv = cv2.cvtColor(frame_gau_blur, cv2.COLOR_BGR2HSV)
    valvesAngleFinder(frame_gau_blur, hsv, frame, color)

    # creates the range of blue
    lower_blue = np.array([110,20,90])
    higher_blue = np.array([130, 255, 255])
    #lower_blue = np.array([110, 20, 90])
    #higher_blue = np.array([130, 148, 190])

    # finds blue regions and extract eblue region dges
    blue_range = cv2.inRange(hsv, lower_blue, higher_blue)
    res_blue = cv2.bitwise_and(frame_gau_blur,frame_gau_blur, mask=blue_range)
    blue_s_gray = cv2.cvtColor(res_blue, cv2.COLOR_BGR2GRAY)
    #canny_edge = cv2.Canny(blue_s_gray, 95, 140)

    # applys HoughCircles
    circles = cv2.HoughCircles(blue_s_gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=2000, param1=10, param2=50, minRadius=80, maxRadius=250)
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cv2.circle(frame, (x,y), r, (0,255,0), 4) # draws circle
            cv2.rectangle(frame, (x-5, y-5), (x+5, y+5), (0,128,255), -1) # draws circle center 
        depthFinder(blue_range, np.pi*radius**2, np.pi*r**2, coef, pow)

    cv2.imshow('Circular Valve', frame)
    #cv2.imshow('blue elements', blue_s_gray)


def leverRect(coef, pow, W, L, frame, type):
    # creates the range of color
    if type == 'lever':
        lower_color = np.array([100, 20, 90])
        higher_color = np.array([120, 148, 255])
        lower_size = 8000
        upper_size = 100000
    else:
        lower_color = np.array([10, 100, 20])
        higher_color = np.array([20, 255, 255])
        lower_size = 1500
        if type == 'small switch':
            upper_size = 100000
        else:
            upper_size = 100000

    # image pre-processing
    frame_gau_blur = cv2.GaussianBlur(frame, (3, 3), 0)
    hsv = cv2.cvtColor(frame_gau_blur, cv2.COLOR_BGR2HSV)

    # finds blue regions and extract eblue region dges
    color_range = cv2.inRange(hsv, lower_color, higher_color)
    res_color = cv2.bitwise_and(frame_gau_blur,frame_gau_blur, mask=color_range)
    color_s_gray = cv2.cvtColor(res_color, cv2.COLOR_BGR2GRAY)

    _, color_s_gray = cv2.threshold(color_s_gray, 8, 255, cv2.THRESH_BINARY)
    cnts = cv2.findContours(color_s_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # Use index [-2] to be compatible to OpenCV 3 and 4

    upper = L/W + 0.2
    lower = L/W - 0.2

    for c in cnts:
        rect = cv2.minAreaRect(c)
        (x,y),(w,h), a = rect
        if (lower < w/(h+0.0001) < upper or lower < h/(w+0.0001) < upper) and lower_size<w*h<upper_size: # filter out incorrect rectangle
            box = cv2.boxPoints(rect)
            box = np.int0(box) #turn into ints
            cv2.drawContours(frame, [box], 0, (0,255,0),4)
            depthFinder(color_range, W*L, w*h, coef, pow)
            angle = round(a, 2)
            if h<w:
                ori = 'horizontal'
                #if angle < 90.00:
                #   angle += 90.00
            else:
                ori = 'vertical'
            #if a < 90.00:
            #    a += 90.00
            #print('Angle:', angle)
            print('Orientation:', ori)

    cv2.imshow('Lever', frame)
    cv2.imshow('blue elements', color_s_gray)


# calculates white pixel numbers and convert to depth info
def depthFinder(white, area, pix_area, coef, pow):
    #number_of_white_pix = np.sum(white == 255)
    ratio = pix_area/area
    distance = round(coef*ratio**(pow), 2)
    #print(ratio)
    print("Distance:", distance, "cm")


def main():
    target = 'small switch'

    cap = cv2.VideoCapture(0)
    if not cap: print("!!!Failed VideoCapture: invalid camera source!!!")

    while(True):
        _, frame = cap.read()
        if type(frame) == type(None):
            print("!!!Couldn't read frame!!!")
            break

        if target == 'small valve':
            coef = 2951.8
            pow = -0.608
            radius = 2.25
            valvesFinder(coef, pow, radius, frame, target)
        elif target == 'large valve': 
            coef = 3025.7
            pow = -0.617
            radius = 5
            valvesFinder(coef, pow, radius, frame, target)
        elif target == "lever": 
            coef = 1034.7
            pow = -0.548
            W = 1.3
            L = 4.6
            leverRect(coef, pow, W, L, frame, target)
        elif target == "small switch":
            coef = 1934.7
            pow = -0.548
            W = 4
            L = 8
            leverRect(coef, pow, W, L, frame, target)
        elif target == "large switch":
            coef = 1934.7
            pow = -0.548
            W = 4
            L = 16
            leverRect(coef, pow, W, L, frame, target)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"): break

    cap.release()
    cv2.destroyAllWindows()
    

if __name__ == "__main__":
    main()