import tifffile
import struct
import numpy as np
from PIL import Image

# Load the TIF
tif_path = "src/telerobotics_sim/models/terrains/lv1_test_site_1_200m_200m_5cmpp_flipped.tif"
heightmap = tifffile.imread(tif_path)

print(f"Original shape: {heightmap.shape}, dtype: {heightmap.dtype}")

# OPTIONAL: Crop to top-left 1/4 for performance
# Uncomment to use:
heightmap = heightmap[:1000, :1000]
print(f"Cropped to: {heightmap.shape}")

# ============ MUJOCO FORMAT ============
output_path = tif_path.replace('.tif', '.hfield')
with open(output_path, 'wb') as f:
    f.write(struct.pack('<i', heightmap.shape[0]))  # nrow
    f.write(struct.pack('<i', heightmap.shape[1]))  # ncol
    f.write(heightmap.astype(np.float32).tobytes())

file_size_mb = (heightmap.shape[0] * heightmap.shape[1] * 4 + 8) / (1024**2)
print(f"MuJoCo .hfield: {file_size_mb:.1f} MB")

# ============ UNITY FORMAT ============

# Normalize to 0-1 while preserving full precision
heightmap_normalized = (
    (heightmap - heightmap.min()) /
    (heightmap.max() - heightmap.min())
).astype(np.float32)

# Resize to 2^n + 1 (Unity constraint)
TARGET = 1025  # Options: 513, 1025, 2049, 4097

# Resize in floating point
pil_img = Image.fromarray(heightmap_normalized, mode="F")
pil_img_resized = pil_img.resize((TARGET, TARGET), Image.BICUBIC)
arr_resized = np.array(pil_img_resized, dtype=np.float32)

# Coordinate transform (same as before)
arr_final = np.rot90(arr_resized, 3)

# Convert to full 16-bit precision
arr16 = np.clip(arr_final * 65535.0, 0, 65535).astype("<u2")

# Save .raw
unity_path = tif_path.replace('.tif', '.raw')
arr16.tofile(unity_path)

print(f"Unity .raw: {arr16.nbytes / (1024**2):.1f} MB")