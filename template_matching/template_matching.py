import cv2 as cv
from os.path import join

def main():
    folder_path = "image"
    img = join(folder_path, "spade-terminal.png")
    tempelate = join(folder_path, "template.png")

    # reads image
    img = cv.imread(img)
    template = cv.imread(tempelate)

    # converts to gray-scale
    img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    template_gray = cv.cvtColor(template, cv.COLOR_BGR2GRAY)
    
    # binary
    _, img_binary = cv.threshold(img_gray, 60, 255, cv.THRESH_BINARY)
    _, template_binary = cv.threshold(template_gray, 60, 255, cv.THRESH_BINARY)

    # clean-up
    for i in range(1):
        img_binary = cv.erode(img_binary, None)
    for i in range(1):
        img_binary = cv.dilate(img_binary, None)
    
    # finds contours in the image and the template
    img_binary = cv.bitwise_not(img_binary)
    temp_binary = cv.bitwise_not(template_binary)
    cont_img, _ = cv.findContours(img_binary, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    cont_temp, _ = cv.findContours(temp_binary, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    # matches the objects in the image with the template
    defect = []
    for i in range(len(cont_img)):
        ctr = cont_img[i]
        dist = cv.matchShapes(ctr, cont_temp[0], cv.CONTOURS_MATCH_I2, 0)
        
        if dist > 1.5: # threshold for defect
            defect.append(ctr)
            
    cv.drawContours(img, defect, -1, (0,0,255), cv.FILLED) # draws defects
    cv.imshow('Defects', img)
    cv.imwrite("results/spade-terminal-output.png", img)
    cv.waitKey()
    cv.destroyAllWindows()
    

if __name__ == "__main__":
    main()
