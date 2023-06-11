import cv2
import matplotlib.pyplot as plt
import numpy as np

path = r'pipeline.jpeg'
img = cv2.imread(path)

h = img.shape[0]
w = img.shape[1]
img = cv2.resize(img, (h//2, w//2))
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)


contours, hierarchy = cv2.findContours(thresh, 
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

mask = np.zeros_like(img)

for contour in contours:
    contour_area = cv2.contourArea(contour)
    
    # Check if contour area is greater than 5000
    if contour_area > 500:
        # Draw the contour on the original image
        cv2.drawContours(mask, [contour], -1, (0, 255, 255), thickness=cv2.FILLED)
mask2 = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
lines = cv2.HoughLines(mask2, rho=1, theta=np.pi / 180, threshold=100)

x1=0
x2=0
y1=0
y2=0

# Iterate over the detected lines
for line in lines:
    rho, theta = line[0]

    # Convert polar coordinates to Cartesian coordinates
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho

    # Calculate the start and end points of the line to draw
    x1 = int(x0 + 1000 * (-b))
    y1 = int(y0 + 1000 * (a))
    x2 = int(x0 - 1000 * (-b))
    y2 = int(y0 - 1000 * (a))

    # Draw the line on the mask image

    cv2.line(mask2, (x1, y1), (x2, y2), (0, 0, 255), 2)

cv2.circle(mask, ((-x1+x2)//2, (-y1+y2)//2), 10, (0,0,255), -1)
print(-x1+x2,-y1+y2)
# Calculate the angle of the detected line
angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi

print(angle)  
cv2.imshow("Pipeline", mask)
cv2.waitKey(0)
cv2.destroyAllWindows()