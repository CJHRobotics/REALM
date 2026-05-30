from torchvision import models
import torch.nn as nn
from torchvision import transforms
import numpy as np
import torch

class FeatureExtractor:
    def __init__(self, cnn_extractor_model):
        self.cnn_extractor_model = cnn_extractor_model

        # Define a dictionary mapping model names to their feature extractors
        model_dict = {
            "resnet50": lambda: nn.Sequential(
                *list(models.resnet50(weights=models.ResNet50_Weights.DEFAULT).children())[:-2]),
            "mobilenetv2": lambda: models.mobilenet_v2(weights=models.MobileNet_V2_Weights.DEFAULT).features,
            "mobilenetv3": lambda: models.mobilenet_v3_small(
                weights=models.MobileNet_V3_Small_Weights.DEFAULT).features
        }

        if self.cnn_extractor_model in model_dict:
            self.cnn_feature_extractor = model_dict[self.cnn_extractor_model]()
            # Freeze the model parameters to prevent training
            for param in self.cnn_feature_extractor.parameters():
                param.requires_grad = False
        else:
            raise ValueError(f"Unsupported CNN extractor model: {self.cnn_extractor_model}")
        self.cnn_feature_extractor.eval()  # Set the model to evaluation mode
        # Define preprocessing (same for both meshes since they are pre-trained on ImageNet)
        self.preprocess = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

    def get_cnn_features(self, pov_image):
        """
        Process the robot's POV image using ResNet50 to extract CNN features.

        Args:
            pov_image (list): The robot's POV image as a 3D list of RGB values.

        Returns:
            np.ndarray: The extracted CNN features as a flattened numpy array.
        """
        # Convert the Webots image (list format) to a numpy array
        pov_image_np = np.array(pov_image, dtype=np.uint8)
        # Apply preprocessing transformations
        pov_image_tensor = self.preprocess(pov_image_np).unsqueeze(0)  # Add batch dimension
        with torch.no_grad():
            features = self.cnn_feature_extractor(pov_image_tensor)
        return features.flatten().numpy()