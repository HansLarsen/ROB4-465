import cv2
import numpy as np
import glob


def find_biggest_contour(image):
    image = image.copy()
    contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
    biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
    bowl = np.zeros(image.shape, np.uint8)
    cv2.drawContours(bowl, [biggest_contour], -1, 255, -1)

    return biggest_contour, bowl

def draw_center(image, center):

    radius = 5
    color = (0, 0, 0)
    thickness = -1
    cv2.circle(image, center, radius, color, thickness)


def bowl_finder(image):
    #resize so all picture are same size
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    max_dimension = max(image.shape)
    scale = 700/max_dimension
    image = cv2.resize(image, None, fx = scale, fy = scale)

    #blur

    image_blur = cv2.GaussianBlur(image,(7,7), 0)
    hsv_image_blur = cv2.cvtColor(image_blur, cv2.COLOR_RGB2HSV)

    #threshold
    min_red = np.array([0, 170, 40])
    max_red = np.array([20, 256,256])

    #color threshold
    filter1 = cv2.inRange(hsv_image_blur, min_red, max_red)

    #color threshold 2
    min_intens_red = np.array([170, 170, 40])
    max_intens_red = np.array([180, 256, 256])
    filter2 = cv2.inRange(hsv_image_blur, min_intens_red, max_intens_red)

    filter = filter1 + filter2

    #segementation
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))
    morph_close = cv2.morphologyEx(filter, cv2.MORPH_CLOSE, kernel )
    morph_open = cv2.morphologyEx(morph_close, cv2.MORPH_OPEN, kernel)

    biggest_contour, bowl = find_biggest_contour(morph_open)

    #find center

    M = cv2.moments(biggest_contour)

    center = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]) )
    #cx = int(M["m10"]/M["m00"])
    #cy = int(M["m01"]/M["m00"])

    draw_center(image, center)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    return bowl, image, center

image = cv2.imread("/home/ubuntu/Desktop/bowl_pics/Bowl_Movements/36.jpg")

bowl, image, center = bowl_finder(image)

print(center)
cv2.imshow("bowl", bowl)
cv2.imshow("test", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
