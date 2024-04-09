from ActiveRequirementLearning import queryable_expert
import numpy as np
from MaddenActiveLearningRequirements import madden_dataset
import csv


class MaddenExpert(queryable_expert.QueryableExpert):
    def __init__(self, postion=None, archetype=None, file_location="Data", overall_formula_filename="Madden_19_overall_formulas.csv",):
        self.file_location = file_location
        self.overall_formula_filename = overall_formula_filename
        self.stat_labels = []
        self.archetype_names = []
        self.position_names = []
        self.overall_formulas = []
        self.position = postion
        self.archetype = archetype
        # initialize member variable that deal with player info
        # construct the filename
        filename_stats = "/".join([self.file_location, self.overall_formula_filename])

        # open the file and get the data
        with open(filename_stats) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            # go through each line of the file and grab the needed data
            current_row = 0
            for current_row, row in enumerate(csv_reader):
                # the first row contains the stat labels
                if current_row == 0:
                    self.stat_labels = row[8:]
                else:
                    self.position_names.append(row[0])
                    self.archetype_names.append(row[1])
                    self.overall_formulas.append([float(i)/100 for i in row[2:-2]])
        self.formula_dict = {}
        for index, archetype in enumerate(self.archetype_names):
            if self.position_names[index] in self.formula_dict:
                self.formula_dict[self.position_names[index]][archetype] = self.overall_formulas[index]
            else:
                self.formula_dict[self.position_names[index]] = {}
                self.formula_dict[self.position_names[index]][archetype] = self.overall_formulas[index]

    def get_query_label(self, features, position=None, archetype=None):
        if position is None and archetype is None:
            position = self.position
            archetype = self.archetype
        if archetype and position is None:
            raise ValueError("Cannot provide archetype without postion")
        elif position and archetype is None:
            best_overall = 0
            for archetype in self.formula_dict[position].keys():
                formula = self.formula_dict[position][archetype]
                overall = np.sum(np.multiply(features, formula))
                if best_overall < overall:
                    best_overall = overall
            return best_overall

        formula = self.formula_dict[position][archetype]
        return np.sum(np.multiply(features, formula))


if __name__ == "__main__":
    position = "CB"
    archetype = "Man to Man"
    expert = MaddenExpert(position, archetype)
    dataset = madden_dataset.MaddenDataset()


    dataset = dataset.get_test_train_validate_set(single_position=position, single_archytype=archetype)["train_set"]
    print(dataset.player_names[0])
    print(expert.get_query_label(dataset.get_features()[0]))
    position = "QB"
    archetype = "Field General"
    print(expert.get_query_label(dataset.get_features()[0], position))
    print(expert.get_query_label(dataset.get_features()[0], position, archetype))
    print("Hello")

