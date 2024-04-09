"""
Active regression on madden dataset with Gaussian processes.
"""
import modAL
import sklearn.metrics
import warnings
import math
import pickle

from ActiveRequirementLearning import active_requirements_learning as ARL, query_strategies as QS, \
    queryable_expert as QE, synthetic_data_creator as SynC, stopping_criteria as SC, \
    active_requirement_learning_dataset as ARLD
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import WhiteKernel, RBF, DotProduct, Matern

from ActiveRequirementLearning.stopping_criteria import CountStopping
from MaddenActiveLearningRequirements import madden_dataset, madden_oracle


def get_estimator():
    # defining the kernel for the Gaussian process
    kernel = DotProduct() + WhiteKernel()
    #kernel= kernel = RBF() + WhiteKernel()
    kernel = 1.0 * Matern(length_scale=100.0, nu=10.5)
    estimator = GaussianProcessRegressor(kernel=kernel)
    return estimator

def learn_on_everything(total_addtional_examples, initial_datasets, file_index):
    pos_dataset = madden_dataset.MaddenDataset(file_location="MaddenActiveLearningRequirements/Data")
    positions = pos_dataset.positions
    single_archytype= None
    print(f"Number of Postions: {len(positions)}")
    print(f"Positions {positions}")
    total_examples_needed = 0
    total_examples = 0
    stopping_size = 0
    statistics = {}
    for index, single_position in enumerate(positions):
        madden_data = madden_dataset.MaddenDataset(file_location="MaddenActiveLearningRequirements/Data")
        initial_dataset = initial_datasets[index]
        full_dataset = madden_dataset.MaddenDataset(file_location="MaddenActiveLearningRequirements/Data",
                                                    position=single_position,
                                                    archetype=single_archytype)
        full_split = full_dataset.get_test_train_validate_set(test_size= 0.001, validate_size= 0.05)
        stopping_dataset = full_split["validate_set"]
        stopping_size += len(stopping_dataset.get_features())

        oracle = madden_oracle.MaddenExpert(postion=single_position, archetype=single_archytype, file_location="MaddenActiveLearningRequirements/Data")
        madden_active_learner = ARL.ActiveRequirementsLearning(estimator=get_estimator(),
                                                               query_strategy=QS.RandomQuery(),
                                                               query_expert=oracle,
                                                               stopping_criteria=CountStopping(number_of_samples=total_addtional_examples[index]),
                                                               initial_training_dataset=initial_dataset,
                                                               stopping_dataset=stopping_dataset,
                                                               use_synthetic_datapoints=False,
                                                               synthetic_data_creator=SynC.NullCreator(),
                                                               is_committee=False)

        stats = madden_active_learner.train_actively(full_dataset)

        labels, std = madden_active_learner.get_prediction(full_dataset.get_features())
        true_labels = full_dataset.get_labels()
        running_sum = 0
        max_difference = 0
        for i, label in enumerate(labels):
            difference = abs(true_labels[i] - label)
            if difference > max_difference:
                max_difference = difference
            running_sum += difference
        MSE = sklearn.metrics.mean_squared_error(true_labels, labels)
        R2 = sklearn.metrics.r2_score(true_labels, labels)

        average_error =  running_sum/len(labels)
        print(f"{single_position} Random Learning Stats")
        print(f"    Total Running Error= {running_sum}")
        print(f"    Mean Absolute Error= {average_error}")
        print(f"    Root Mean Squared Error= {math.sqrt(MSE)}")
        print(f"    R^2 Error= {R2}")
        print(f"    Total Additional Examples Needed={len(madden_active_learner.current_dataset.get_features())- len(madden_active_learner.initial_training_dataset.get_features())}")
        print(f"    Initial Number of Examples={len(madden_active_learner.initial_training_dataset.get_features())}")
        total_examples += len(madden_active_learner.current_dataset.get_features())
        total_examples_needed += len(madden_active_learner.current_dataset.get_features()) - len(madden_active_learner.initial_training_dataset.get_features())
        print(f"    Stats: {stats}")
        statistics[single_position] = stats
    print(f"Total additional samples needed={total_examples_needed}")
    statistics["Total_Additional_Samples"] = total_examples_needed

    print(f"Total samples needed={total_examples}")
    statistics["Total_Samples"] = total_examples

    total_with_stopping = total_examples_needed + stopping_size
    print(f"Total Required Labeled Examples Including Stopping Dataset: {total_with_stopping}")
    statistics["Total_Samples_w_Stopping"] = total_with_stopping

    print(f"Total Possible Elements: {len(madden_data.get_features())*len(positions)}")
    statistics["Total_Possible"] = len(madden_data.get_features())*len(positions)

    print(f"Total Possible Elements Per Postion: {len(madden_data.get_features())}")
    statistics["Total_Possible_Per_Position"] = len(madden_data.get_features())
    filename = f"total_learning_stats_5_{file_index}.pickle"
    pickle.dump(statistics, open(filename, "wb"))

def active_learn():
    pos_dataset = madden_dataset.MaddenDataset(file_location="MaddenActiveLearningRequirements/Data")
    positions = pos_dataset.positions
    single_archytype= None
    print(f"Number of Postions: {len(positions)}")
    print(f"Positions {positions}")
    total_examples_needed = 0
    total_examples = 0
    stopping_size = 0
    statistics = {}
    total_additional_examples = []
    init_sets = []
    for single_position in positions:
        stop = 0.90
        if single_position == "QB":
            stop = 0.99
        madden_data = madden_dataset.MaddenDataset(file_location="MaddenActiveLearningRequirements/Data")
        initial_dataset = madden_data.get_test_train_validate_set(single_position=single_position, single_archytype=single_archytype, test_size= 0.02, validate_size=0.02, emptysets=True)["test_set"]
        init_sets.append(initial_dataset)
        full_dataset = madden_dataset.MaddenDataset(file_location="MaddenActiveLearningRequirements/Data",
                                                    position=single_position,
                                                    archetype=single_archytype)
        full_split = full_dataset.get_test_train_validate_set(test_size= 0.001, validate_size= 0.10)
        stopping_dataset = full_split["validate_set"]
        stopping_size += len(stopping_dataset.get_features())

        oracle = madden_oracle.MaddenExpert(postion=single_position, archetype=single_archytype, file_location="MaddenActiveLearningRequirements/Data")
        madden_active_learner = ARL.ActiveRequirementsLearning(estimator=get_estimator(),
                                                               query_strategy=QS.ModalStrategyRegressor(modal_partial=modAL.disagreement.max_std_sampling),
                                                               query_expert=oracle,
                                                               stopping_criteria=SC.MetricStopping(metric=sklearn.metrics.r2_score, stopping_score=stop, score_higher=True),
                                                               initial_training_dataset=initial_dataset,
                                                               stopping_dataset=stopping_dataset,
                                                               use_synthetic_datapoints=False,
                                                               synthetic_data_creator=SynC.NullCreator(),
                                                               is_committee=False)

        stats = madden_active_learner.train_actively(full_dataset)

        labels, std = madden_active_learner.get_prediction(full_dataset.get_features())
        true_labels = full_dataset.get_labels()
        running_sum = 0
        max_difference = 0
        for i, label in enumerate(labels):
            difference = abs(true_labels[i] - label)
            if difference > max_difference:
                max_difference = difference
            running_sum += difference
        MSE = sklearn.metrics.mean_squared_error(true_labels, labels)
        R2 = sklearn.metrics.r2_score(true_labels, labels)

        average_error =  running_sum/len(labels)
        print(f"{single_position} Active Learning Stats")
        print(f"    Total Running Error= {running_sum}")
        print(f"    Mean Absolute Error= {average_error}")
        print(f"    Root Mean Squared Error= {math.sqrt(MSE)}")
        print(f"    R^2 Error= {R2}")
        print(f"    Total Additional Examples Needed={len(madden_active_learner.current_dataset.get_features())- len(madden_active_learner.initial_training_dataset.get_features())}")
        total_additional_examples.append(len(madden_active_learner.current_dataset.get_features())- len(madden_active_learner.initial_training_dataset.get_features()))
        print(f"    Initial Number of Examples={len(madden_active_learner.initial_training_dataset.get_features())}")
        total_examples += len(madden_active_learner.current_dataset.get_features())
        total_examples_needed += len(madden_active_learner.current_dataset.get_features()) - len(madden_active_learner.initial_training_dataset.get_features())
        print(f"    Stats: {stats}")
        statistics[single_position] = stats
    print(f"Total additional samples needed={total_examples_needed}")
    statistics["Total_Additional_Samples"] = total_examples_needed

    print(f"Total samples needed={total_examples}")
    statistics["Total_Samples"] = total_examples

    total_with_stopping = total_examples_needed + stopping_size
    print(f"Total Required Labeled Examples Including Stopping Dataset: {total_with_stopping}")
    statistics["Total_Samples_w_Stopping"] = total_with_stopping

    print(f"Total Possible Elements: {len(madden_data.get_features())*len(positions)}")
    statistics["Total_Possible"] = len(madden_data.get_features())*len(positions)

    print(f"Total Possible Elements Per Postion: {len(madden_data.get_features())}")
    statistics["Total_Possible_Per_Position"] = len(madden_data.get_features())

    pickle.dump(statistics, open("active_learning_stats_v5.pickle", "wb"))
    return total_additional_examples, init_sets

if __name__ == "__main__":
    warnings.filterwarnings("ignore")
    run_active = True
    if(run_active):
        total_additional_examples, initial_dataset = active_learn()
        pickle.dump(total_additional_examples, open("total_additional.pickle", "wb"))
        pickle.dump(initial_dataset, open("init_data.pickle", "wb"))
    for i in range(20):
        print(f"Index={i}")
        total_additional_examples = pickle.load(open("total_additional.pickle", "rb"))
        initial_dataset = pickle.load(open("init_data.pickle", "rb"))
        learn_on_everything(total_additional_examples, initial_dataset, i)

