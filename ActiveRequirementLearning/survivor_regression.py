"""
Active regression example with Gaussian processes. Using a survivor domain
"""
from ActiveRequirementLearning import active_requirements_learning as ARL, query_strategies as QS, \
    queryable_expert as QE, synthetic_data_creator as SynC, stopping_criteria as SC, \
    active_requirement_learning_dataset as ARLD
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import WhiteKernel, RBF
from MaddenActiveLearningRequirements import madden_dataset, madden_oracle


def get_estimator():
    # defining the kernel for the Gaussian process
    kernel = RBF(length_scale=1.0, length_scale_bounds=(1e-2, 1e3)) \
             + WhiteKernel(noise_level=1, noise_level_bounds=(1e-10, 1e+1))
    estimator = GaussianProcessRegressor(kernel=kernel)
    return estimator


def main():
    initial_dataset = madden_dataset.MaddenDataset()
    full_dataset = madden_dataset.MaddenDataset()

    survivor_active_learner = ARL.ActiveRequirementsLearning(estimator=get_estimator(),
                                                             query_strategy=QS.GPRegressionStd(),
                                                             query_expert=madden_oracle.MaddenExpert(),
                                                             stopping_criteria=SC.OverallUncertaintyStopping(),
                                                             initial_training_dataset=initial_dataset,
                                                             use_synthetic_datapoints=True,
                                                             synthetic_data_creator=SynC.ExampleCreator(
                                                                 sample_amount=100),
                                                             is_committee=False)

    survivor_active_learner.train_actively(full_dataset)


if __name__ == "__main__":
    main()
