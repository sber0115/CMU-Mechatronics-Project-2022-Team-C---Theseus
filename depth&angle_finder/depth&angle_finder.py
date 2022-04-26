import cv2
import matplotlib.pyplot as plt
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
        sensitivity = 20
        lower_hsv = np.array([0, 0, 255-sensitivity])
        higher_hsv = np.array([255, sensitivity, 255])
        upper = 12/10+0.2
        lower = 12/10-0.2
        upper_size = 7000
        lower_size = 1000

    # finds blue regions and extract eblue region dges
    hsv_range = cv2.inRange(hsv, lower_hsv, higher_hsv)
    res_hsv = cv2.bitwise_and(frame_gau_blur,frame_gau_blur, mask=hsv_range)
    hsv_s_gray = cv2.cvtColor(res_hsv, cv2.COLOR_BGR2GRAY)

    _, hsv_s_gray = cv2.threshold(hsv_s_gray, 8, 255, cv2.THRESH_BINARY)
    cnts = cv2.findContours(hsv_s_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # Use index [-2] to be compatible to OpenCV 3 and 4

    x_rec = 0
    y_rec = 0
    angle = 0
    pre_angle = 0
    for c in cnts:
        rect = cv2.minAreaRect(c)
        (x,y),(w,h), a = rect
        if (lower < w/(h+0.0001) < upper or lower < h/(w+0.0001) < upper) and upper_size>w*h>lower_size: # filter out incorrect rectangle
            pre_angle = angle
            box = cv2.boxPoints(rect)
            box = np.int0(box) #turn into ints
            x_rec = round(np.int0(x))
            y_rec = round(np.int0(y))
            cv2.drawContours(frame, [box], 0, (0,0,255), 4)
            cv2.rectangle(frame, (x_rec-5, y_rec-5), (x_rec+5, y_rec+5), (0,128,255), -1) # draws circle center
            angle = round(a, 2)
            if abs(angle-pre_angle) > 15 and angle == 270:
                angle = pre_angle
    
    #cv2.imshow('Circular Valve', frame)
    # cv2.imshow('white elements', hsv_s_gray)

    return x_rec, y_rec, angle


def valvesFinder(coef, pow, radius, frame, type):
    # select marker type
    if type == 'small valve':
        color = 'green'
    else:
        color = 'white'

    # image pre-processing
    frame_gau_blur = cv2.GaussianBlur(frame, (3, 3), 0)
    hsv = cv2.cvtColor(frame_gau_blur, cv2.COLOR_BGR2HSV)
    x_angle, y_angle, angle = valvesAngleFinder(frame_gau_blur, hsv, frame, color)

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
            x = round(x, 2)
            y = round(y, 2)
            print("Circle Center: ({}, {})".format(x, y))
            # print("Pointer Center: ({}, {})".format(x_angle, y_angle))

            if x_angle > x:
                if y_angle > y:
                    angle = angle + 90.00
            else:
                if y_angle > y:
                    angle = 180 + angle
                else:
                    angle = 270 + angle

            print('Angle:', angle)

        depthFinder(blue_range, np.pi*radius**2, np.pi*r**2, coef, pow)
        
    cv2.imshow('Circular Valve', frame)
    #cv2.imshow('blue elements', blue_s_gray)q


def leverRect(coef, pow, W, L, frame, type):
    # creates the range of color
    if type == 'lever':
        lower_color = np.array([110,20,70])
        higher_color = np.array([130, 255, 255])
        lower_size = 8000
        upper_size = 100000
    else:
        lower_color = np.array([12, 100, 90])
        higher_color = np.array([20, 255, 255])
        lower_size = 1000
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

            #depthFinder(color_range, W*L, w*h, coef, pow)
            angle = round(a, 2)
            x = round(x, 2)
            y = round(y, 2)
            if h<w:
                ori = 'horizontal'
            else:
                ori = 'vertical'

            if not type == 'lever':
                if a<5:
                    continue
                elif 90-a > 5:
                    continue
                elif ori == 'horizontal':
                    continue
                else:
                    cv2.drawContours(frame, [box], 0, (0,255,0),4)
                    print("Center: ({}, {})".format(x, y))
            else:
                cv2.drawContours(frame, [box], 0, (0,255,0),4)
                x = np.int0(x)
                y = np.int0(y)
                cv2.rectangle(frame, (x-5, y-5), (x+5, y+5), (0,128,255), -1) # draws circle center
                print('Orientation:', ori)
                print("Center: ({}, {})".format(x, y))

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
    target = 'large valve'

    cap = cv2.VideoCapture(1)
    if not cap: print("!!!Failed VideoCapture: invalid camera source!!!")

    while(True):
        _, frame = cap.read()

        # adaptive histogram flattening
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))    # flatten value
        v_cor = clahe.apply(v)
        hsv = cv2.merge((h, s, v_cor))
        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        # norm = np.zeros(frame.shape)
        # frame = cv2.normalize(frame, norm, 0, 255, cv2.NORM_MINMAX)
        # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # h, s, v = cv2.split(hsv)

        # plt.hist(v_cor.ravel(), 256, [0, 256])
        # plt.show()

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
            W = 1.6
            L = 5.6
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
            W = 0.8
            L = 3.2
            leverRect(coef, pow, W, L, frame, target)
        else:
            print("!!!Wrong Target!!!")
            break

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"): break

    cap.release()
    cv2.destroyAllWindows()
    

if __name__ == "__main__":
    main()