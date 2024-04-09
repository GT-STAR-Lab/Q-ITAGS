# Collection of Helper functions for grabbing Madden Data and Processing it
import csv
from sklearn.model_selection import train_test_split
import sys
import MaddenActiveLearningRequirements

class MaddenDataset():
    def __init__(self, init_file=True,
                 position=None,
                 archetype=None,
                 file_location="Data",
                 player_stat_filename="Madden_19_players_stats.csv",
                 overalls_filename="Madden_19_overalls_by_pos.csv",
                 init_subset_from_existing=False,
                 to_init_from=None,
                 player_indices=[]):
        # File Information
        self.file_location = file_location
        self.player_stat_filename = player_stat_filename
        self.overalls_filename = overalls_filename

        # Player Info
        self.overall_position = position
        self.overall_archetype = archetype
        self.player_names = []           # list of player names (string)
        self.player_team = []            # list of teams each player plays for (e.g. patriots, ect.) (string)
        self.player_positions = []       # list of player positions (e.g. QB, RB, ect.) (string)
        self.player_stats = []           # list of the stats for each player (ints)
        self.stat_labels = []            # list of the names for each stat (e.g. speed, strength, ect.) (string)
        self.player_archetype = []       # list of player sub_position archetype (e.g. power rusher) (string)

        # Performance Info
        self.overalls = []                  # list of player overall rankings for each position and archtype
        self.overall_position_name = []     # labels for the postion columns for overalls
        self.overall_archetype_name = []    # labels for the archetype columns for overalls
        self.player_positions_overall = []  # overall score for a player in the postion that they play

        # Game Info
        self.positions = {}
        self.archetypes = {}

        # Init Fields
        if init_file and not init_subset_from_existing:
            self.get_overall_info()
            self.get_player_info()
            self.set_game_info()
            if self.overall_archetype is None:
                self.get_player_archetype()
            else:
                self.get_specific_postion_archetype()
        elif not init_file and init_subset_from_existing and to_init_from and len(player_indices):
            self.create_subset_from_existing_dataset(to_init_from, player_indices)
            self.set_game_info()
        else:
            raise Exception("Invalid Init Parameter")

    def create_subset_from_existing_dataset(self, to_init_from, player_indices):
        self.stat_labels = to_init_from.stat_labels
        self.overall_position_name = to_init_from.overall_position_name
        self.overall_archetype_name = to_init_from.overall_archetype_name

        for i in player_indices:
            # Player Info
            self.player_names.append(to_init_from.player_names[i])
            self.player_team.append(to_init_from.player_team[i])
            self.player_positions.append(to_init_from.player_positions[i])
            self.player_stats.append(to_init_from.player_stats[i])
            self.player_archetype.append(to_init_from.player_archetype[i])

            # Performance Info
            self.overalls.append(to_init_from.overalls[i])
            self.player_positions_overall.append(to_init_from.player_positions_overall[i])

    def set_game_info(self):
        self.positions = set(self.player_positions)
        self.archetypes = set(self.overall_archetype_name)

    def get_player_info(self):
        # initialize member variable that deal with player info
        # construct the filename
        filename_stats = "/".join([self.file_location, self.player_stat_filename])

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
                    self.player_names.append(row[2])
                    self.player_team.append(row[0])
                    if self.overall_position is None:
                        self.player_positions.append(row[4])
                    else:
                        self.player_positions.append(self.overall_position)
                    self.player_stats.append([float(i) for i in row[8:]])

    def get_specific_postion_archetype(self):
        archetype_index = None
        for index, archetype_name in enumerate(self.overall_archetype_name):
                if archetype_name == self.overall_archetype and self.overall_position_name[index] == self.overall_position:
                    archetype_index = index
                    break

        for player_index, player in enumerate(self.player_names):
                player_position = self.player_positions[player_index]
                best_overall_value = 0
                best_overall_archetype = None

                self.player_archetype.append(self.overall_archetype)
                self.player_positions_overall.append(self.overalls[player_index][archetype_index])
        
    def get_player_archetype(self):
        for player_index, player in enumerate(self.player_names):
            player_position = self.player_positions[player_index]
            best_overall_value = 0
            best_overall_archetype = None
            for archetype_index, overall in enumerate(self.overalls[player_index]):
                overall = float(overall)
                if self.overall_position_name[archetype_index] == self.player_positions[player_index]:
                    if overall > best_overall_value:
                        best_overall_value = overall
                        best_overall_archetype = self.overall_archetype_name[archetype_index]
            self.player_archetype.append(best_overall_archetype)
            self.player_positions_overall.append(best_overall_value)

    def get_overall_info(self):
        # initialize member variable that deal with player info
        # construct the filename
        filename_overalls = "/".join([self.file_location, self.overalls_filename])

        with open(filename_overalls) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            # go through each line of the file and grab the needed data
            for current_row, row in enumerate(csv_reader):
                # First row contains position labels
                if current_row == 0:
                    self.overall_position_name = row[2:]
                # Second row contains archetype labels
                elif current_row == 1:
                    self.overall_archetype_name = row[2:]
                # Other rows contain player overall info
                else:
                    self.overalls.append([float(i) for i in row[2:]])

    def get_test_train_validate_set(self, single_position=None, single_archytype=None, test_size= 0.10, validate_size= 0.15, emptysets=False):
        # split the data into test, train, validate sets
        # single position and single archtype only return overall values for a specific postion or archetype
        # real_postion_only will only return a subset of the dataset with players that actually play the specified postion

        indices = self.get_indices_of_players(single_position, single_archytype)
        train_indices, test_validate_indices = train_test_split(indices, test_size=(test_size+validate_size))
        validate_indices, test_indices = train_test_split(test_validate_indices, test_size=(test_size/(test_size+validate_size)))

        if(not emptysets):
            train_set = MaddenDataset(init_file=False, init_subset_from_existing=True, to_init_from=self, player_indices=train_indices)
            test_set = MaddenDataset(init_file=False, init_subset_from_existing=True, to_init_from=self, player_indices=test_indices)
            validate_set = MaddenDataset(init_file=False, init_subset_from_existing=True, to_init_from=self, player_indices=validate_indices)
        else:
            train_set = MaddenDataset(init_file=False, init_subset_from_existing=True, to_init_from=self, player_indices=[0])
            test_set = MaddenDataset(init_file=False, init_subset_from_existing=True, to_init_from=self, player_indices=[0])
            validate_set = MaddenDataset(init_file=False, init_subset_from_existing=True, to_init_from=self, player_indices=[0])

        test_train_validate_dict = {"train_set": train_set, "test_set": test_set, "validate_set": validate_set}

        return test_train_validate_dict

    def get_indices_of_players(self, single_position=None, single_archytype=None,):
        indices = []
        if not single_position and not single_archytype:
            indices = range(len(self.player_names))
        elif single_position and not single_archytype:
            for index, player_position in enumerate(self.player_positions):
                if player_position == single_position:
                    indices.append(index)
        elif single_position and single_archytype:
            for index, player_archetype in enumerate(self.player_archetype):
                if player_archetype == single_archytype and self.player_positions[index] == single_position:
                    indices.append(index)
        else:
            raise ValueError("If using single archetype must provide postion")
        return indices

    def get_features(self):
        return self.player_stats

    def get_labels(self):
        return self.player_positions_overall

    def add_datapoint(self, example_features, label):
        for index in range(len(example_features)):
            self.player_stats.append(example_features[index])
            self.player_positions_overall.append(label[index])
            self.player_positions.append(self.player_positions_overall)
            self.player_archetype.append(self.overall_archetype_name)





# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    dataset = MaddenDataset()
    split_set1 = dataset.get_test_train_validate_set()
    split_set2 = dataset.get_test_train_validate_set(single_position="QB")
    split_set3 = dataset.get_test_train_validate_set(single_position="K", single_archytype="Power")
    print("Done")
