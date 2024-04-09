from sklearn.model_selection import train_test_split
import numpy as np
from abc import ABC, abstractmethod
import random

from MaddenActiveLearningRequirements import madden_dataset


class SyntheticDataCreatorBase(ABC):
    def __init__(self, recreate=False, sample_amount=None, sample_percentage=None):
        self.sample_amount = sample_amount
        self.sample_percentage = sample_percentage
        self.synthetic_datapoints = []
        self.recreate = recreate

    def add_synthetic_datapoints(self, datapoints):
        if self.recreate or len(self.synthetic_datapoints)==0:
            if self.sample_amount:
                for i in range(self.sample_amount):
                    datapoints.append(self.create_synthetic_datapoint(datapoints))
            elif self.sample_percentage:
                for i in range(len(datapoints)*self.sample_percentage):
                    datapoints.append(self.create_synthetic_datapoint(datapoints))
            else:
                raise ValueError("Need to Specify either sample percentage or amount")
        else:
            datapoints.extend(self.synthetic_datapoints)
        return datapoints

    @abstractmethod
    # query strategy for regression
    def create_synthetic_datapoint(self):
        pass


class NullCreator(SyntheticDataCreatorBase):
    def create_synthetic_datapoint(self, datapoints):
        return

    def add_synthetic_datapoints(self, datapoints):
        return datapoints


class RangeCreator(SyntheticDataCreatorBase):
    def __init__(self, ranges, recreate=False, sample_amount=None, sample_percentage=None):
        self.ranges = ranges
        self.sample_amount = sample_amount
        self.sample_percentage = sample_percentage

    def create_synthetic_datapoint(self, datapoints):
        synthetic_point = []
        for range_sample in self.ranges:
            synthetic_point.append(random.randrange(range_sample[0], range_sample[1]))
        return synthetic_point


class GaussianCreator(SyntheticDataCreatorBase):
    def create_synthetic_datapoint(self, datapoints):
        std = list(np.std(datapoints, axis=0))
        mean = list(np.average(datapoints, axis=0))
        synthetic_point = list(np.random.normal(mean, std, (len(mean))))
        return synthetic_point


class ExampleCreator(SyntheticDataCreatorBase):
    def create_synthetic_datapoint(self, datapoints):
        synthetic_point = []
        for index in range(len(datapoints[0])):
            selected = random.randrange(0, len(datapoints))
            synthetic_point.append(datapoints[selected][index])
        return synthetic_point


if __name__ == "__main__":
    position = "CB"
    archetype = "Man to Man"
    dataset = madden_dataset.MaddenDataset(file_location="../MaddenActiveLearningRequirements/Data")
    datapoints = dataset.get_test_train_validate_set(single_position=position, single_archytype=archetype, paired_examples=True)["train_set"].get_features()

    exp_creator = ExampleCreator(sample_amount=1)
    new_datapoint_1 = exp_creator.create_synthetic_datapoint(datapoints)
    gauss_creator = GaussianCreator(sample_amount=1)
    new_datapoint_2 = gauss_creator.create_synthetic_datapoint(datapoints)

    ranges = [[0,100] for i in range(53)]
    range_creator = RangeCreator(sample_amount=1, ranges=ranges)
    new_datapoint_3 = range_creator.create_synthetic_datapoint()


