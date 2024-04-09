import numpy as np
import sklearn
from modAL import CommitteeRegressor, ActiveLearner
import pickle
import copy
import math


class ActiveRequirementsLearning:
    def __init__(self, estimator, query_strategy, query_expert, stopping_criteria, initial_training_dataset, stopping_dataset=None, use_synthetic_datapoints=False, synthetic_data_creator=None, is_committee=False):
        if is_committee:
            self.regressor = CommitteeRegressor(
                learner_list=estimator,
                query_strategy=query_strategy.get_query_index
            )
        else:
            # initializing the active learner
            self.regressor = ActiveLearner(
                estimator=estimator,
                query_strategy=query_strategy.get_query_index,
                X_training=initial_training_dataset.get_features(), y_training=initial_training_dataset.get_labels())
        self.stopping_dataset = stopping_dataset
        self.use_current_stopping = False
        if stopping_dataset is None:
            self.stopping_dataset = initial_training_dataset
            self.use_current_stopping = True
        self.stop_criteria = stopping_criteria
        self.initial_training_dataset = initial_training_dataset
        self.current_dataset = copy.deepcopy(initial_training_dataset)
        self.query_expert = query_expert
        self.use_synthetic_datapoints = use_synthetic_datapoints
        self.synthetic_data_creator = synthetic_data_creator
        self.train_statistics = {
            "stopping_dataset_metrics": {"rmse": [], "r2e": [], "mae":[]},
            "query_dataset_metric": {"rmse": [], "r2e": [], "mae": []},
            "current_dataset_metrics": {"rmse": [], "r2e": [], "mae": [], "initial_size": len(self.current_dataset.get_labels()), "final_size": 0},
            "initial_dataset_metrics": {"rmse": [], "r2e": [], "mae": []}
        }

    def get_regressor_query(self, datapoints):
        query_idx, query_instance = self.regressor.query(datapoints)
        return query_idx, query_instance

    def teach_regressor(self, example_features, label):
        self.current_dataset.add_datapoint(example_features, label)
        if self.use_current_stopping:
            self.stopping_dataset.add_datapoint(example_features, label)
        self.regressor.teach(example_features, label)


    def create_synthetic_data(self, datapoints):
        dataset = self.synthetic_data_creator.add_synthetic_datapoints(datapoints)
        return dataset

    def train_actively(self, query_dataset):
        print("train actively")
        query_features = query_dataset.get_features()
        if self.use_synthetic_datapoints:
            query_features = self.create_synthetic_data(query_features)
        self.add_training_round_to_stats(query_dataset)
        while not self.stop_criteria.is_finished(self, self.stopping_dataset):
            query_idx, query_instance = self.get_regressor_query(query_features)
            label = self.query_expert.get_query_label(query_instance)
            self.teach_regressor(query_instance, [label])
            self.add_training_round_to_stats(query_dataset)
        self.train_statistics["current_dataset_metrics"]["final_size"] = len(self.current_dataset.get_labels())
        return self.train_statistics

    def add_training_round_to_stats(self, query_dataset):
        self.get_and_store_stats(query_dataset, "query_dataset_metric")
        self.get_and_store_stats(self.initial_training_dataset, "initial_dataset_metrics")
        self.get_and_store_stats(self.current_dataset, "current_dataset_metrics")
        self.get_and_store_stats(self.stopping_dataset, "stopping_dataset_metrics")
        return

    def get_and_store_stats(self, dataset, name):
        r2, mea, rmse = self.get_stats_values(dataset)
        self.train_statistics[name]["rmse"].append(rmse)
        self.train_statistics[name]["r2e"].append(r2)
        self.train_statistics[name]["mae"].append(mea)

    def get_stats_values(self, dataset):
        true_labels = []
        predicted_labels = []
        for feature in dataset.get_features():
            true_label = self.query_expert.get_query_label(feature)
            pred_label, _ = self.get_prediction([feature])
            true_labels.append(true_label)
            predicted_labels.append(pred_label)
        r2 = self.get_r_squared(true_labels, predicted_labels)
        mae = self.get_mean_absolute_error(true_labels, predicted_labels)
        rmse = self.get_root_mean_squared_error(true_labels, predicted_labels)
        return r2, mae, rmse

    def get_r_squared(self, true_labels, predicted_labels):
        return sklearn.metrics.r2_score(true_labels, predicted_labels)

    def get_mean_absolute_error(self, true_labels, predicted_labels):
        mae = sklearn.metrics.mean_absolute_error(true_labels, predicted_labels)
        return mae

    def get_root_mean_squared_error(self, true_labels, predicted_labels):
        return math.sqrt(sklearn.metrics.mean_squared_error(true_labels, predicted_labels))

    def get_score(self, datapoints, labels):
        return self.regressor.score(datapoints, labels)

    def get_prediction(self, datapoint):
        return self.regressor.predict(datapoint, return_std=True)

    def get_min_score(self):
        return np.min(self.get_prediction(self.initial_training_dataset))

    def save_model(self, filename):
        pickle.dump(self, open(filename, 'wb'))


def load_active_requirements_learning(filename):
    loaded_model = pickle.load(open(filename, 'rb'))
    return loaded_model
