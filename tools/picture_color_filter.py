% picture color filtering

PATH = "picture/h1.png"
image1 = cv2.imread(PATH)
cv_rgb1 = cv2.cvtColor(image1, cv2.COLOR_BGR2RGB)
plt.imshow(cv_rgb1)
plt.show()

hsv1 = cv2.cvtColor(image1, cv2.COLOR_BGR2HSV)
# define range of blue color in HSV
lower = np.array([10,180,50])
upper = np.array([30,255,255])
# Threshold the HSV image to get only blue colors
mask = cv2.inRange(hsv1, lower, upper)
rgb_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
plt.imshow(rgb_mask)
plt.show()

kernel = np.ones((5,5),np.uint8)
dilation = cv2.dilate(mask,kernel,iterations = 2)

opening = cv2.morphologyEx(dilation, cv2.MORPH_OPEN, kernel)
rgb = cv2.cvtColor(opening, cv2.COLOR_GRAY2RGB)
plt.imshow(rgb)
plt.show()

kernel = np.ones((19,19),np.uint8)

opening = cv2.morphologyEx(dilation, cv2.MORPH_OPEN, kernel)
rgb = cv2.cvtColor(opening, cv2.COLOR_GRAY2RGB)
plt.imshow(rgb)
plt.show()
