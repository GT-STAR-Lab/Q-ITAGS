from sklearn.model_selection import train_test_split
import numpy as np
from abc import ABC, abstractmethod


class StoppingCriteriaBase(ABC):
    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    # query strategy for regression
    def is_finished(self, regressor, dataset):
       pass


class CountStopping(StoppingCriteriaBase):
    def __init__(self, number_of_samples):
        self.max_example = number_of_samples
        self.current_num_examples = 0

    def is_finished(self, regressor, dataset):
        if self.max_example > self.current_num_examples:
            self.current_num_examples +=1
            return False
        return True


class ScoreStopping(StoppingCriteriaBase):
    def __init__(self, stopping_score, score_higher=True):
        self.stopping_score = stopping_score
        self.score_higher = score_higher

    def is_finished(self, regressor, dataset):
        score = regressor.get_score(dataset.get_features(), dataset.get_labels())
        if self.score_higher and score > self.stopping_score:
            return True
        elif not self.score_higher and score < self.stopping_score:
            return True
        return False


class MetricStopping(StoppingCriteriaBase):
    def __init__(self, metric, stopping_score, score_higher=True):
        #metric must be a partial that takes 2 params y_true=labels, y_pred=dataset.get_labels()
        self.metric = metric
        self.stopping_score = stopping_score
        self.score_higher = score_higher

    def is_finished(self, regressor, dataset):
        labels, std = regressor.get_prediction(dataset.get_features())
        true_labels = dataset.get_labels()
        score = self.metric(y_true=labels, y_pred=true_labels)
        if self.score_higher and score > self.stopping_score:
            return True
        elif not self.score_higher and score < self.stopping_score:
            return True
        return False


class OverallUncertaintyStopping(StoppingCriteriaBase):
    def __init__(self, max_uncertainty):
        self.max_uncertainty = max_uncertainty

    def is_finished(self, regressor, dataset):
        labels, std = regressor.get_prediction(dataset.get_features())
        current_average_uncertainty = np.sum(std)/len(std)
        if self.max_uncertainty > current_average_uncertainty:
            return True
        return False


class MaxUncertaintyStopping(StoppingCriteriaBase):
    def __init__(self, max_uncertainty):
        self.max_uncertainty = max_uncertainty

    def is_finished(self, regressor, dataset):
        labels, std = regressor.get_prediction(dataset.get_features())
        current_max_uncertainty = np.amax(std)
        if self.max_uncertainty > current_max_uncertainty:
            return True
        return False
