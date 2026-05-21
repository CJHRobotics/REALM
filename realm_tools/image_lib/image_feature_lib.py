import cv2
import numpy as np


def extract_hog_features(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    hog = cv2.HOGDescriptor(_winSize=(64, 128),
                            _blockSize=(16, 16),
                            _blockStride=(8, 8),
                            _cellSize=(8, 8),
                            _nbins=9)
    resized_image = cv2.resize(gray_image, (64, 128))
    hog_features = hog.compute(resized_image).flatten()
    return hog_features


def extract_color_histogram(image, bins=(8, 8, 8)):
    hist = cv2.calcHist([image], [0, 1, 2], None, bins, [0, 256, 0, 256, 0, 256])
    return cv2.normalize(hist, hist).flatten()


def extract_spatial_histogram(image, size=(32, 32)):
    downscaled_image = cv2.resize(image, size).flatten()
    return downscaled_image


def extract_combined_features(image, landmark_mask=None, robot_theta=None):
    # Ensure the image is in the correct format
    image = np.array(image, dtype=np.uint8)[:, :, :3]
    # Extract individual features
    hog_features = extract_hog_features(image)
    color_histogram = extract_color_histogram(image)
    spatial_histogram = extract_spatial_histogram(image)
    if landmark_mask is not None and robot_theta is not None:
        # Concatenate all features into a single flat feature vector
        combined_features = np.concatenate([hog_features, color_histogram, spatial_histogram, landmark_mask, [robot_theta]])
    else:
        combined_features = np.concatenate([hog_features, color_histogram, spatial_histogram])
    return combined_features

