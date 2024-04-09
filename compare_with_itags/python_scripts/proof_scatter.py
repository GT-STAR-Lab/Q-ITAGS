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

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.ticker import FormatStrFormatter

# Editable Fields

number_of_problems = 8

# Directory Data
problems_directory = "/outputs_icra/outputs_proof/"
base_subfolder = "Base"
alt_subfolder = "alts/"
output_subfolder = "./outputs_icra/Output_proof"
problem_base_name = "survivor_problem"

alphas = ["0.010000", "0.100000", "0.200000", "0.300000", "0.400000", "0.500000", "0.600000", "0.700000", "0.800000",
          "0.900000", "1.000000"]
problem_nums = [0,1,3,11,12,14,17,18,20,24,25, 26, 27, 28, 30, 31, 32, 33, 34, 35, 37, 38, 39, 41, 42, 44, 45, 47, 48, 49, 50, 51,52,55, 58, 59, 60,61,62, 63, 64]

def main():
    average = []
    labels = []
    differences = []
    x = 11
    fig, ax = plt.subplots(1, 1)
    plt.suptitle(f"Validation of Bounds on Allocation Quality")
    ax.set_xlabel('Alpha Value')
    ax.set_ylabel('Normalized Allocation-optimality gap')

    all_makespan_data = []
    all_allocation_data = []
    for problem_nr in problem_nums:
        makespan_data = []
        allocation_data = []
        for alpha in alphas:
            path_out_base = f'{output_subfolder}/{alpha}{problem_base_name}{problem_nr}.json'

            with open(path_out_base) as f:
                data = json.load(f)
                allocation_data.append(data["allocation_quality"])
                makespan_data.append(data["solution"]["makespan"])
        all_makespan_data.append(makespan_data)
        all_allocation_data.append(allocation_data)

    violins = []
    for i in range(len(alphas)):
        alpha_range = []
        for index, prob in enumerate(all_makespan_data):
            alpha_range.append((prob[0] - prob[i]) / (prob[0] - min(prob) + 1e-4))
        violins.append(alpha_range)
    float_alpha = [0.01, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
    ax.violinplot(violins[0:x],  widths=0.3,
                  showmeans=False, showmedians=False, showextrema=False)

    for i in range(len(all_makespan_data)):
        data = [(datum - all_makespan_data[i][0]) / (all_makespan_data[i][-1] - all_makespan_data[i][0]) for datum in
                all_makespan_data[i]]
        if i == len(all_makespan_data) - 1:
            plt.scatter(alphas[0:x], data[0:x], color='cadetblue', label="Actual optimality gap")
        else:
            plt.scatter(alphas[0:x], data[0:x], color='cadetblue')

    data = []
    for j in alphas:
        if (float(j) == 0):
            data.append(0)
        elif float(j) < 0.5:
            data.append(float(j) / (1 - float(j)))
        else:
            data.append(1)

    plt.ylim([0, 1.1])
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    ax.xaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    plt.plot(alphas[0:x], data[0:x], label=f"Theoretical bound")
    plt.legend()

    plt.show()


if __name__ == '__main__':
    main()
