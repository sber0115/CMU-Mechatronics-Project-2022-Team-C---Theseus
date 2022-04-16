import cv2
import numpy as np


def valvesAngleFinder(frame_gau_blur, hsv, frame, color):
    # creates the range of green
    if color == 'green':
        lower_hsv = np.array([25, 52, 72])
        higher_hsv = np.array([102, 255, 255])
    else:
        lower_hsv = np.array([25, 52, 72])
        higher_hsv = np.array([102, 255, 255])

    # finds blue regions and extract eblue region dges
    hsv_range = cv2.inRange(hsv, lower_hsv, higher_hsv)
    res_hsv = cv2.bitwise_and(frame_gau_blur,frame_gau_blur, mask=hsv_range)
    hsv_s_gray = cv2.cvtColor(res_hsv, cv2.COLOR_BGR2GRAY)

    _, hsv_s_gray = cv2.threshold(hsv_s_gray, 8, 255, cv2.THRESH_BINARY)
    cnts = cv2.findContours(hsv_s_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # Use index [-2] to be compatible to OpenCV 3 and 4

    for c in cnts:
        rect = cv2.minAreaRect(c)
        (x,y),(w,h), a = rect
        if (1.8 < w/(h+0.0001) < 2.5 or 1.8 < h/(w+0.0001) < 2.5) and w*h>4000: # filter out incorrect rectangle
            box = cv2.boxPoints(rect)
            box = np.int0(box) #turn into ints
            cv2.drawContours(frame, [box], 0, (0,0,255), 4)
            angle = round(a, 2)
            print('Angle:', angle)
    #cv2.imshow('Circular Valve', frame)
    #cv2.imshow('blue elements', white_s_gray)


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
    lower_blue = np.array([110, 50, 50])
    higher_blue = np.array([130, 255, 255])

    # finds blue regions and extract eblue region dges
    blue_range = cv2.inRange(hsv, lower_blue, higher_blue)
    res_blue = cv2.bitwise_and(frame_gau_blur,frame_gau_blur, mask=blue_range)
    blue_s_gray = cv2.cvtColor(res_blue, cv2.COLOR_BGR2GRAY)
    canny_edge = cv2.Canny(blue_s_gray, 95, 140)

    # applys HoughCircles
    circles = cv2.HoughCircles(canny_edge, cv2.HOUGH_GRADIENT, dp=1.2, minDist=2000, param1=10, param2=50, minRadius=80, maxRadius=250)
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cv2.circle(frame, (x,y), r, (0,255,0), 4) # draws circle
            cv2.rectangle(frame, (x-5, y-5), (x+5, y+5), (0,128,255), -1) # draws circle center 
        depthFinder(blue_range, np.pi*radius**2, coef, pow)

    cv2.imshow('Circular Valve', frame)
    #cv2.imshow('blue elements', blue_s_gray)


def leverRect(coef, pow, W, L, frame, type):
    # creates the range of color
    if type == 'lever':
        lower_color = np.array([110, 50, 50])
        higher_color = np.array([130, 255, 255])
        lower_size = 8000
        upper_size = 100000
    else:
        lower_color = np.array([161, 155, 84])
        higher_color = np.array([179, 255, 255])
        lower_size = 100
        if type == 'small switch':
            upper_size = 500
        else:
            upper_size = 1000

    # image pre-processing
    frame_gau_blur = cv2.GaussianBlur(frame, (3, 3), 0)
    hsv = cv2.cvtColor(frame_gau_blur, cv2.COLOR_BGR2HSV)

    # finds blue regions and extract eblue region dges
    color_range = cv2.inRange(hsv, lower_color, higher_color)
    res_color = cv2.bitwise_and(frame_gau_blur,frame_gau_blur, mask=color_range)
    color_s_gray = cv2.cvtColor(res_color, cv2.COLOR_BGR2GRAY)

    _, color_s_gray = cv2.threshold(color_s_gray, 8, 255, cv2.THRESH_BINARY)
    cnts = cv2.findContours(color_s_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # Use index [-2] to be compatible to OpenCV 3 and 4

    upper = L/W + 0.1
    lower = L/W - 0.1

    for c in cnts:
        rect = cv2.minAreaRect(c)
        (x,y),(w,h), a = rect
        if (lower < w/(h+0.0001) < upper or lower < h/(w+0.0001) < upper) and lower_size<w*h<upper_size: # filter out incorrect rectangle
            box = cv2.boxPoints(rect)
            box = np.int0(box) #turn into ints
            cv2.drawContours(frame, [box], 0, (0,255,0),4)
            depthFinder(color_range, W*L, coef, pow)
            angle = round(a, 2)
            print('Angle:', angle)

    cv2.imshow('Lever', frame)
    #cv2.imshow('blue elements', blue_s_gray)


# calculates white pixel numbers and convert to depth info
def depthFinder(white, area, coef, pow):
    number_of_white_pix = np.sum(white == 255)
    if number_of_white_pix<2000:
        distance = 100000000
    else:
        ratio = number_of_white_pix/area
        distance = round(coef*ratio**(pow), 2)
        #print(ratio)
        print("Distance:", distance, "cm")


def main():
    target = 'small valve'

    cap = cv2.VideoCapture(0)
    if not cap: print("!!!Failed VideoCapture: invalid camera source!!!")

    while(True):
        _, frame = cap.read()
        if type(frame) == type(None):
            print("!!!Couldn't read frame!!!")
            break

        if target == 'small valve':
            coef = 1034.7
            pow = -0.548
            radius = 5
            #valvesAngleFinder(frame)
            valvesFinder(coef, pow, radius, frame, target)
        elif target == 'large valve': 
            coef = 1934.7
            pow = -0.548
            radius = 12
            valvesFinder(coef, pow, radius, frame, target)
        elif target == "lever": 
            coef = 1034.7
            pow = -0.548
            W = 4.3
            L = 8.7
            leverRect(coef, pow, W, L, frame, target)
        elif target == "small switch":
            coef = 1934.7
            pow = -0.548
            W = 3
            L = 3
            leverRect(coef, pow, W, L, frame, target)
        elif target == "large switch":
            coef = 1934.7
            pow = -0.548
            W = 3
            L = 7
            leverRect(coef, pow, W, L, frame, target)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"): break

    cap.release()
    cv2.destroyAllWindows()
    

if __name__ == "__main__":
    main()