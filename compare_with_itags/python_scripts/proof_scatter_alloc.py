import json
import numpy as np
import matplotlib
from matplotlib import pyplot as plt
from matplotlib.ticker import FormatStrFormatter
matplotlib.use('TkAgg')
# number_of_problems = 8

# Data directory
output_subfolder = './outputs_icra/Output_proof'
problem_base_name = "survivor_problem"

alphas = ["0.010000", "0.100000", "0.200000", "0.300000", "0.400000", "0.500000", "0.600000", "0.700000", "0.800000",
          "0.900000", "1.000000"]
problem_nums = [0, 1, 3, 11, 12, 14, 17, 18, 20, 24, 25, 26, 27, 28, 30, 31, 32, 33, 34, 35, 37, 38, 39, 41, 42, 44, 45,
                47, 48, 49, 50, 51, 52, 55, 58, 59, 60, 61, 62, 63, 64]


def main():
    x = 11
    # set the font family to Times New Roman and adjust the font size at the VERY BEGINNING
    SMALL_SIZE = 14
    MEDIUM_SIZE = 14
    BIGGER_SIZE = 16
    #### FONT SIZE ###########################################################
    plt.rc('font', size=MEDIUM_SIZE)  # controls default text sizes
    plt.rc('axes', titlesize=MEDIUM_SIZE)  # fontsize of the axes title
    plt.rc('axes', labelsize=MEDIUM_SIZE)  # fontsize of the x and y labels
    plt.rc('xtick', labelsize=MEDIUM_SIZE)  # fontsize of the tick labels
    plt.rc('ytick', labelsize=MEDIUM_SIZE)  # fontsize of the tick labels
    plt.rc('legend', fontsize=SMALL_SIZE)  # legend fontsize
    plt.rc('figure', titlesize=MEDIUM_SIZE)  # fontsize of the figure title
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
    plt.rcParams['text.usetex'] = True
    plt.rcParams["figure.autolayout"] = True

    fig, ax = plt.subplots(1, 1, figsize=(5, 4))
    plt.suptitle(f"Validation of Bounds on Allocation Quality", x=0.55, y=0.97)
    # ax.set_xlabel('Alpha Value')
    ax.set_xlabel(r"$\alpha$ value")
    ax.set_ylabel(r'Normalized Allocation Optimality Gap')

    all_allocation_data = []
    for problem_nr in problem_nums:
        allocation_data = []
        for alpha in alphas:
            path_out_base = f'{output_subfolder}/{alpha}{problem_base_name}{problem_nr}.json'

            with open(path_out_base) as f:
                data = json.load(f)
                allocation_data.append(data["allocation_quality"])
        all_allocation_data.append(allocation_data)

    # TODO: violins
    violins = []
    for i in range(len(alphas)):
        alpha_range = []
        for index, prob in enumerate(all_allocation_data):
            alpha_range.append((max(prob) - prob[i])/(max(prob) - min(prob)))
        violins.append(alpha_range)
    float_alpha = [0.01, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
    ax.violinplot(violins[0:x], widths=0.3, showmeans=False, showmedians=False, showextrema=False)

    for i in range(len(all_allocation_data)):
        data = [(max(all_allocation_data[i]) - datum) / (max(all_allocation_data[i]) - min(all_allocation_data[i])) for
                datum in all_allocation_data[i]]
        if i == len(all_allocation_data) - 1:
            plt.scatter(alphas[0:x], data[0:x], color='cadetblue', label="Actual optimality gap")
        else:
            plt.scatter(alphas[0:x], data[0:x], color='cadetblue')

    data = []
    for j in alphas:
        if float(j) == 0:
            data.append(0)
        elif float(j) < 0.5:
            data.append(float(j) / (1 - float(j)))
        else:
            data.append(1)

    plt.ylim([0, 1.05])
    custom_tick_labels = ['0.0', '0.1', '0.2', '0.3', '0.4', '0.5', '0.6', '0.7', '0.8', '0.9', '1.0']
    ax.set_xticklabels(custom_tick_labels[0:x])
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    # ax.xaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    plt.plot(alphas[0:x], data[0:x], color='orange', label=f"Theoretical bound")
    plt.legend(loc='upper left')
    # plt.subplots_adjust(left=0.05, right=0.9, top=0.99, bottom=0.2)
    # plt.show()
    plt.tight_layout()
    plt.savefig('images/proof_bound.pdf')

if __name__ == "__main__":
    main()
