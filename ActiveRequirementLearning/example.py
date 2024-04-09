"""
Example of Active regression example with Gaussian processes. Using a sin wave dataset
"""
import numpy as np
import matplotlib.pyplot as plt
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import WhiteKernel, RBF
from ActiveRequirementLearning import active_requirements_learning as ARL, query_strategies as QS, \
    queryable_expert as QE, synthetic_data_creator as SynC, stopping_criteria as SC,  \
    active_requirement_learning_dataset as ARLD
from sklearn import linear_model
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import WhiteKernel, RBF
from modAL.models import ActiveLearner, CommitteeRegressor
from modAL.disagreement import max_std_sampling

class SinExpert(QE.QueryableExpert):
    def get_query_label(self, features):
        return np.sin(features)


class sin_dataset(ARLD.ActiveRequirementLearningDataset):
    def __init__(self, size):
            # generating the data
            self.X = np.random.choice(np.linspace(0, 20, 10000), size=size, replace=False).reshape(-1, 1)
            self.y = np.sin(self.X)

    def get_features(self):
        return self.X

    def get_labels(self):
        return self.y

    def add_datapoint(self, example_features, label):
        np.append(self.X, example_features)
        np.append(self.y, label)


def plot_prediction(arl, title, dataset):
    # plotting the initial estimation
    with plt.style.context('seaborn-white'):
        plt.figure(figsize=(14, 7))
        x = np.linspace(0, 20, 1000)
        pred, std = arl.get_prediction(x.reshape(-1,1))
        plt.plot(x, pred)
        plt.fill_between(x, pred.reshape(-1, )-std, pred.reshape(-1, )+std, alpha=0.2)
        plt.scatter(dataset.get_features(), dataset.get_labels(), c='k')
        plt.title(title)
        plt.waitforbuttonpress()
        plt.show()


def show_performance(arl, dataset, n_initial, n_queries):
    #plot_prediction(arl, 'Initial estimation based on %d points' % n_initial, dataset)
    arl.train_actively(dataset)
    plot_prediction(arl, 'Estimation after %d queries' % n_queries, dataset)


def get_estimator(dataset):
    # defining the kernel for the Gaussian process
    kernel = RBF(length_scale=1.0, length_scale_bounds=(1e-2, 1e3)) \
             + WhiteKernel(noise_level=1, noise_level_bounds=(1e-10, 1e+1))
    estimator = GaussianProcessRegressor(kernel=kernel)
    return estimator


def main():
    n_initial = 1
    n_queries = 10
    arl_v1 = ARL.ActiveRequirementsLearning(get_estimator(sin_dataset(n_initial)), QS.GPRegressionStd(), SinExpert(), SC.CountStopping(n_queries), sin_dataset(n_initial), use_synthetic_datapoints=True, synthetic_data_creator=SynC.NullCreator())
    show_performance(arl_v1, sin_dataset(50), n_initial, n_queries)



if __name__ == "__main__":
  main()


# GETR COLLECTION OF ESTIMATORS
