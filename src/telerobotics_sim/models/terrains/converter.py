from PIL import Image
import numpy as np

img = Image.open("testpit_5cmpp_gray.png").convert("L")
arr = np.array(img, dtype=np.uint8)  # MuJoCo-validated orientation, untouched

# Resample ONCE to a valid Unity resolution (513x513 is the nearest above 400)
TARGET = 513
img_resized = Image.fromarray(arr).resize((TARGET, TARGET), Image.BICUBIC)
arr_resized = np.array(img_resized, dtype=np.uint8)

Image.fromarray(arr_resized).save("testpit_5cmpp_gray_513.png")

# Now apply the transpose (needed for Unity's raw reader) from this shared source
arr_t = arr_resized.T
arr16 = (arr_t.astype(np.uint16) * 257).astype("<u2")
arr16.tofile("testpit_5cmpp_gray.raw")