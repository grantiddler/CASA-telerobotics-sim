from PIL import Image
import numpy as np

# Load image
img = Image.open("testpit_5cmpp_gray.png").convert("L")

# Convert to 16-bit
arr = np.array(img, dtype=np.uint8)
arr = arr.T
arr16 = (arr.astype(np.uint16) * 257)   # 0-255 -> 0-65535

# Unity expects little-endian
arr16 = arr16.astype("<u2")

arr16.tofile("testpit_5cmpp_gray.raw")