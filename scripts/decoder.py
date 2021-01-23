import numpy as np


""" 
function to replace self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
returns a numpy array reshaped by (height, width, 3)
"""
def decodeImage(data, height, width): 
    decoded = np.fromstring(data, dtype=np.uint8)
    decoded = decoded.reshape((height, width, 3))
    return decoded
