#  Graphically Recursive Simultaneous Task Allocation, Planning,
#  Scheduling, and Execution
#
#  Copyright (C) 2020-2022
#
#  Author: Andrew Messing
#  Author: Glen Neville
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.


import json
import os
from os.path import exists

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
# Editable Fields
from matplotlib.colors import LinearSegmentedColormap

number_of_problems = 50

sub_height = 1
sub_width = 2

# Directory Data
output_subfolder = "/outputs_icra/comp"
problem_base_name = "survivor_problem"

def main():
    SMALL_SIZE = 14
    MEDIUM_SIZE = 14
    BIGGER_SIZE = 16
    #### FONT SIZE ###########################################################
    plt.rc('font', size=MEDIUM_SIZE)  # controls default text sizes
    plt.rc('axes', titlesize=BIGGER_SIZE)  # fontsize of the axes title
    plt.rc('axes', labelsize=MEDIUM_SIZE)  # fontsize of the x and y labels
    plt.rc('xtick', labelsize=MEDIUM_SIZE)  # fontsize of the tick labels
    plt.rc('ytick', labelsize=MEDIUM_SIZE)  # fontsize of the tick labels
    plt.rc('legend', fontsize=SMALL_SIZE)  # legend fontsize
    plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
    plt.rcParams['text.usetex'] = True
    plt.rcParams["figure.autolayout"] = True

    stats = [
        "motion_planning_time",  # 0
        "nodes_deadend",  # 1
        "nodes_evaluated",  # 2
        "nodes_expanded",  # 3
        "nodes_generated",  # 4
        "nodes_pruned",  # 5
        "nodes_reopened",  # 6
        "num_motion_plan_failures",  # 7
        "num_motion_plans",  # 8
        "num_scheduling_failures",  # 9
        "num_scheduling_iterations",  # 10
        "scheduling_time",  # 11
        "task_allocation_time",  # 12
        "total_time",  # 13
        "makespan",  # 14
        "allocation_quality" #15
    ]

    y_axis_names = [
        "Motion Planning Time (s)",
        "Number of Deadend Nodes",
        "Number of Nodes Evaluated",
        "Number of Nodes Expanded",
        "Number of Nodes Generated",
        "Number of Nodes Pruned",
        "Number of Nodes Reopened",
        "Number of Motion Planning Failures",
        "Number of Motion Plans Generated",
        "Number of Scheduling Failures",
        "Number of Scheduling Iterations",
        "Scheduling Time (s)",
        "Task Allocation Time (s)",
        "Total Computation Time",
        "Makespan",
        "Allocation Quality"
    ]

    # print(os.getcwd())
    filenames = next(os.walk(f'{output_subfolder}/'), (None, None, []))[2]  # [] if no file
    # Non editable
    # For holding the data from JSON


    average = []
    labels = []
    differences = []
    fig, ax = plt.subplots(1, 1, figsize=(5, 5))
    ax.set_aspect('equal')

    makespan_base = []
    makespan_itags = []
    allocation_quality_base = []
    allocation_quality_itags = []
    compute_times_base = []
    compute_times_itags = []
    count = 0
    for problem_nr in range(number_of_problems):
        path_out_base = f'./{output_subfolder}/{problem_base_name}{problem_nr}.json'
        path_out_itags= f'./{output_subfolder}/{problem_base_name}{problem_nr}_itags.json'
        if exists(path_out_base) and exists(path_out_itags):
            count += 1
            with open(path_out_base) as f:
                data = json.load(f)
            compute_times_base.append(data["statistics"]["total_time"])

            with open(path_out_itags) as f:
                data = json.load(f)
            compute_times_itags.append(data["statistics"]["total_time"])
    print(np.max(compute_times_base))
    print(len(compute_times_base))

    # plot the computation time
    if len(compute_times_base) > 1:
        cat_make = np.zeros(len(compute_times_base))
        # 0 for q-tags better - green, 1 for equal - grey, 2 for q-itags worse - red
        cat_make[np.array(compute_times_base) < np.array(compute_times_itags)] = 0
        cat_make[np.array(compute_times_base) == np.array(compute_times_itags)] = 1
        cat_make[np.array(compute_times_base) > np.array(compute_times_itags)] = 2
        cat_make = cat_make.astype(np.int32)
        print(cat_make)
        data = pd.DataFrame(
            {"X Value": compute_times_base, "Y Value": compute_times_itags, "Category": cat_make})
        groups = data.groupby("Category")
        colors = [(0, 0.8, 0, 0.5), (0.4, 0.4, 0.4, 0.8), (0.8, 0, 0, 0.5)]
        color_map = []
        for i in cat_make:
            color_map.append(colors[i])

        # ax.scatter(compute_times_itags, compute_times_base, c=color_map)
        for name, group in groups:
            ax.scatter(group["Y Value"], group["X Value"], label='Q-ITAGS $\equiv$ ITAGS' if name == 1 else 'Q-ITAGS $\prec$ ITAGS',
                          color=colors[name])

        ax.set_xlabel('ITAGS')
        ax.set_title(f'Computation Time (Unit: s)')
        ax.set_ylabel('Q-ITAGS')
        ax.set_xlim(0, 1200)
        ax.set_ylim(0, 1200)

        xy_line = np.arange(0, 1200)
        ax.plot(xy_line, '#bbbbbb', ls="--")

    plt.legend(loc="upper right")
    fig.tight_layout()


    # plt.show()
    plt.savefig('images/compare_time.pdf')

if __name__ == '__main__':
    main()