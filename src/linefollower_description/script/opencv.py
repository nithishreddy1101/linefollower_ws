import numpy as np
import cv2

# Create a 500x500 white image (255 is white in grayscale)
white_image = np.ones((1000, 1000, 3), dtype=np.uint8) * 255

# Save as PNG
cv2.imwrite("white_image.png", white_image)
