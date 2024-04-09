from sklearn.model_selection import train_test_split
import numpy as np
from abc import ABC, abstractmethod


class ActiveRequirementLearningDataset:
    @abstractmethod
    def __init__(self, size):
        pass

    @abstractmethod
    def get_features(self):
        pass

    @abstractmethod
    def get_labels(self):
        pass

    @abstractmethod
    def add_datapoint(self, example_features, label):
        pass

