from sklearn.model_selection import train_test_split
import numpy as np
from abc import ABC, abstractmethod
import random

class QueryStrategyBase(ABC):
    @abstractmethod
    # query strategy for regression
    def get_query_index(self, regressor, dataset):
       pass


class GPRegressionStd(QueryStrategyBase):
    def get_query_index(self, regressor, datapoints):
        _, std = regressor.predict(datapoints, return_std=True)
        value = np.argmax(std)
        return [value]

class GPRegressionStdMin(QueryStrategyBase):
    def get_query_index(self, regressor, datapoints):
        _, std = regressor.predict(datapoints, return_std=True)
        value = np.argmin(std)
        return [value]

class RandomQuery(QueryStrategyBase):
    def get_query_index(self, regressor, datapoints):
        value = random.randint(0, 0)
        if value == 0:
            _, std = regressor.predict(datapoints, return_std=True)
            pos = random.randrange(0, 80)
            value = np.percentile(std, pos, method="closest_observation")
            return [std.tolist().index(value)]

        value = random.randint(0, len(datapoints)-1)
        return [value]


class ModalStrategy(QueryStrategyBase):
    def __init__(self, modal_partial):
        self.modal_partial = modal_partial

    def get_query_index(self, regressor, datapoints):
        index = self.modal_partial(classifier=regressor, X=datapoints)
        return index


class ModalStrategyCommittee(QueryStrategyBase):
    def __init__(self, modal_partial):
        self.modal_partial = modal_partial

    def get_query_index(self, regressor, datapoints):
        index = self.modal_partial(committee=regressor, X=datapoints)
        return index


class ModalStrategyRegressor(QueryStrategyBase):
    def __init__(self, modal_partial):
        self.modal_partial = modal_partial

    def get_query_index(self, regressor, datapoints):
        index = self.modal_partial(regressor=regressor, X=np.array(datapoints))
        return index


class ModalStrategyEntropy(QueryStrategyBase):
    def __init__(self, modal_partial):
        self.modal_partial = modal_partial

    def get_query_index(self, regressor, datapoints):
        index = self.modal_partial(learner=regressor, X=datapoints)
        return index


class ModalStrategyDensity(QueryStrategyBase):
    def __init__(self, modal_partial):
        self.modal_partial = modal_partial

    def get_query_index(self, regressor, datapoints):
        index = self.modal_partial(X=datapoints)
        return np.max(index)




