#! /usr/bin/env python3
import rclpy

import numpy as np

arr = np.array([1.1, 1.2])
arr_float32 = arr.astype(np.float32)
print(arr, type(arr))
# Convert to Python list
arr = arr_float32.tolist()
print(arr, type(arr))