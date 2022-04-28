import cv2 as cv2
from os.path import join
import numpy as np

folder_path = "templates"

def valvesFinder(coef, pow, radius, frame, type):
    tempelate = join(folder_path, "small_valve_template-canny.png")
    original = frame

    # reads image
    template = cv2.imread(tempelate)

    # converts to gray-scale
    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    
    # binary
    _, img_binary = cv2.threshold(img_gray, 60, 255, cv2.THRESH_BINARY)
    _, template_binary = cv2.threshold(template_gray, 60, 255, cv2.THRESH_BINARY)

    # clean-up
    for i in range(1):
        img_binary = cv2.erode(img_binary, None)
    for i in range(1):
        img_binary = cv2.dilate(img_binary, None)
    
    # finds contours in the image and the template
    img_binary = cv2.bitwise_not(img_binary)
    temp_binary = cv2.bitwise_not(template_binary)
    cont_img, _ = cv2.findContours(img_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cont_temp, _ = cv2.findContours(temp_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # matches the objects in the image with the template
    defect = []
    for i in range(len(cont_img)):
        ctr = cont_img[i]
        dist = cv2.matchShapes(ctr, cont_temp[0], cv2.CONTOURS_MATCH_I2, 0)
        print(dist)
        
        if dist < 10: # threshold for defect
            defect.append(ctr)
    
    results = np.ones(frame.shape)
    cv2.drawContours(results, defect, -1, (0,0,255), cv2.FILLED) # draws defects
    cv2.drawContours(frame, defect, -1, (0,0,255), cv2.FILLED) # draws defects
    results = results.astype(np.uint8)
    results = cv2.cvtColor(results, cv2.COLOR_BGR2GRAY)

    # # select marker type
    # if type == 'small valve':
    #     color = 'green'
    # else:
    #     color = 'white'

    # # image pre-processing
    # frame_gau_blur = cv2.GaussianBlur(frame, (3, 3), 0)
    # hsv = cv2.cvtColor(frame_gau_blur, cv2.COLOR_BGR2HSV)
    # x_angle, y_angle, angle = valvesAngleFinder(frame_gau_blur, hsv, frame, color)

    # # creates the range of blue
    # lower_blue = np.array([110,20,90])
    # higher_blue = np.array([130, 148, 190])
    # #lower_blue = np.array([110, 20, 90])
    # #higher_blue = np.array([130, 148, 190])

    # # finds blue regions and extract eblue region dges
    # blue_range = cv2.inRange(hsv, lower_blue, higher_blue)
    # res_blue = cv2.bitwise_and(frame_gau_blur,frame_gau_blur, mask=blue_range)
    # blue_s_gray = cv2.cvtColor(res_blue, cv2.COLOR_BGR2GRAY)
    # #canny_edge = cv2.Canny(blue_s_gray, 95, 140)

    # applys HoughCircles
    circles = cv2.HoughCircles(results, cv2.HOUGH_GRADIENT, dp=1.2, minDist=2000, param1=10, param2=50, minRadius=80, maxRadius=250)
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cv2.circle(original, (x,y), r, (0,255,0), 4) # draws circle
            cv2.rectangle(original, (x-5, y-5), (x+5, y+5), (0,128,255), -1) # draws circle center
            x = round(x, 2)
            y = round(y, 2)
            print("Circle Center: ({}, {})".format(x, y))
            # print("Pointer Center: ({}, {})".format(x_angle, y_angle))

            # if x_angle > x:
            #     if y_angle > y:
            #         angle = angle + 90.00
            # else:
            #     if y_angle > y:
            #         angle = 180 + angle
            #     else:
            #         angle = 270 + angle

            # print('Angle:', angle)

        #depthFinder(blue_range, np.pi*radius**2, np.pi*r**2, coef, pow)
    cv2.imshow('Circular Valve', original)
    cv2.imshow('Circular Valve', frame)
            


def main():
    target = 'small valve'

    cap = cv2.VideoCapture(1)
    if not cap: print("!!!Failed VideoCapture: invalid camera source!!!")

    while(True):
        _, frame = cap.read()

        # adaptive histogram flattening
        # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # h, s, v = cv2.split(hsv)
        # clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))    # flatten value
        # v_cor = clahe.apply(v)
        # hsv = cv2.merge((h, s, v_cor))
        # frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        norm = np.zeros(frame.shape)
        frame = cv2.normalize(frame, norm, 0, 255, cv2.NORM_MINMAX)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

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
            W = 1.8
            L = 6
            #leverRect(coef, pow, W, L, frame, target)
        elif target == "small switch":
            coef = 1934.7
            pow = -0.548
            W = 4
            L = 8
            #leverRect(coef, pow, W, L, frame, target)
        elif target == "large switch":
            coef = 1934.7
            pow = -0.548
            W = 0.8
            L = 3.2
            #leverRect(coef, pow, W, L, frame, target)
        else:
            print("!!!Wrong Target!!!")
            break

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"): break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
