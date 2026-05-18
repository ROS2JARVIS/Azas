# Role: Small ROS Image <-> NumPy helpers that avoid cv_bridge runtime ABI issues.

from __future__ import annotations

from typing import Optional

import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header


def image_msg_to_cv_image(msg: Image, desired_encoding: str = "passthrough"):
    """Convert common ROS Image encodings into NumPy arrays.

    This intentionally covers the RealSense encodings used by this package:
    rgb8/bgr8 color images and 16UC1/mono16/32FC1 depth images.
    """
    encoding = msg.encoding.lower()

    if encoding in ("rgb8", "bgr8"):
        image = _reshape_image_data(msg, np.uint8, channels=3)
        if desired_encoding == "bgr8" and encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if desired_encoding == "rgb8" and encoding == "bgr8":
            return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image.copy()

    if encoding in ("mono8", "8uc1"):
        return _reshape_image_data(msg, np.uint8, channels=1).copy()

    if encoding in ("mono16", "16uc1", "z16"):
        return _reshape_image_data(msg, np.uint16, channels=1).copy()

    if encoding == "32fc1":
        return _reshape_image_data(msg, np.float32, channels=1).copy()

    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def cv_image_to_image_msg(
    image: np.ndarray,
    encoding: str = "bgr8",
    header: Optional[Header] = None,
) -> Image:
    if image.ndim == 2:
        height, width = image.shape
        channels = 1
    elif image.ndim == 3:
        height, width, channels = image.shape
    else:
        raise ValueError(f"Unsupported image rank: {image.ndim}")

    msg = Image()
    if header is not None:
        msg.header = header
    msg.height = int(height)
    msg.width = int(width)
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = int(width * channels * image.dtype.itemsize)
    msg.data = np.ascontiguousarray(image).tobytes()
    return msg


def _reshape_image_data(msg: Image, dtype, channels: int) -> np.ndarray:
    dtype = np.dtype(dtype)
    row_values = msg.step // dtype.itemsize
    expected_values = msg.width * channels
    data = np.frombuffer(msg.data, dtype=dtype)

    if msg.height <= 0 or msg.width <= 0:
        raise ValueError("Image height/width must be positive.")
    if row_values < expected_values:
        raise ValueError(
            f"Image step is too small for encoding: step={msg.step}, "
            f"width={msg.width}, channels={channels}, dtype={dtype}"
        )

    rows = data.reshape((msg.height, row_values))
    useful = rows[:, :expected_values]
    if channels == 1:
        return useful.reshape((msg.height, msg.width))
    return useful.reshape((msg.height, msg.width, channels))
