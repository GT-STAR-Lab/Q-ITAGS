"""
Active regression on madden dataset with Gaussian processes.
"""
import math
import pickle
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.font_manager as mfm
import numpy as np
matplotlib.use('TkAgg')

dir_prefix = "/home/jliu3103/project/q_itags/backup/"
active_learning_file = "active_learning_stats_v5.pickle"
complete_dataset_learner = "total_learning_stats.pickle"
positions = ['QB','P', 'OG', 'DE', 'K', 'TE', 'OLB', 'S', 'DT', 'OC', 'FB', 'WR', 'CB', 'HB', 'MLB']
position_names = ['Quarterback', 'Punter', 'Off. Guard', 'Def. End', 'Kicker', 'Tight End', 'Outside Linebacker', 'Safety', 'Def. Tackle', 'Off. Center', 'Fullback', 'Wide Receiver', 'Cornerback', 'Halfback', 'Middle Linebacker']
datasets = ['stopping_dataset_metrics', 'query_dataset_metric', 'current_dataset_metrics', 'initial_dataset_metrics']
metrics = ['rmse', 'r2e', 'mae']
# metric_names = ['Root Mean Squared Error', 'Performance Prediction Error', 'Mean Absolute Error']
# metric_names = ['Root Mean Squared Error', 'Coefficient of Determination', 'Mean Absolute Error']
metric_names = ['RMSE', 'Coefficient of Determination', 'Mean Absolute Error']
metrics_to_plot = [0]
print(len(position_names))
positions_to_plot = [3, 6]
positions_to_plot = [14, 2]
positions_to_plot = [7, 10]
datasets_to_plot = [1]

def get_single_graph(axes, active_data, random_data,  random_stat_max, random_stat_min, metric_name, postion_name, index):
    combined_data = active_data + random_data
    axes[index].set_title(postion_name + " Active Learner")
    axes[index].set(xlabel='Number of Datapoints', ylabel=metric_name)
    axes[index].plot(range(len(active_data)), active_data, label="Active Sampling", linewidth=3)
    axes[index].set_ylim(-1, 1.2)


    axes[index].set_title(postion_name)
    axes[index].set(xlabel='Number of Datapoints', ylabel=metric_name)
    axes[index].plot(range(len(random_data)), random_data, label="Uniform Sampling", linewidth=3)
    error = [random_stat_min, random_stat_max]
    #axes[index].errorbar(range(len(random_data)), random_data, error, linestyle='None', marker='')
    axes[index].fill_between(range(len(random_data)), random_stat_max, random_stat_min,
    alpha=0.2, edgecolor='#FF9E45', facecolor='#FF9E45',
    linewidth=4, linestyle='dashdot', antialiased=True)
    axes[index].set_ylim(0, random_data[0]+2)

    if len(random_data) < 15:
        axes[index].set_xticks(range(len(random_data)))
        axes[index].set_xticks(range(len(active_data)))
    elif len(random_data) < 50:
        axes[index].set_xticks(range(0, len(random_data), 5))
        axes[index].set_xticks(range(0, len(active_data), 5))
    elif len(random_data) < 100:
        axes[index].set_xticks(range(0, len(random_data), 10))
        axes[index].set_xticks(range(0, len(active_data), 10))
    else:
        axes[index].set_xticks(range(0, len(random_data), 20))
        axes[index].set_xticks(range(0, len(active_data), 20))

    axes[index].legend()
    axes[index].set_rasterized(True)
    return

def graph_learner_results(filename_active, filename_random, dataset):

    stats_active = pickle.load(open(filename_active, "rb"))
    stats_random = []
    for i in range(19):
        stats_random.append(pickle.load(open(f"total_learning_stats_5_{i}.pickle", "rb")))


    for metric in metrics_to_plot:
        fig, axs = plt.subplots(1, len(positions_to_plot), figsize=(20, 5))
        # fig.suptitle('Active Learning Versus Uniform Sampling')
        for index, position in enumerate(positions_to_plot):
                active_stat = stats_active[positions[position]][datasets[dataset]][metrics[metric]]
                print(active_stat)
                random_stat = []
                for i in range(19):
                    random_stat.append(stats_random[i][positions[position]][datasets[dataset]][metrics[metric]])
                random_stat_average = np.average(np.array(random_stat), axis=0).tolist()
                random_stat_max = np.max(np.array(random_stat), axis=0) #np.subtract(np.max(np.array(random_stat), axis=0), random_stat_average).tolist()
                random_stat_min = np.min(np.array(random_stat), axis=0) #np.subtract(random_stat_average, np.min(np.array(random_stat), axis=0)).tolist()
                print(random_stat)
                get_single_graph(axs, active_stat, random_stat_average, random_stat_max, random_stat_min, metric_names[metric], position_names[position], index)

    return


def graph_results():
    SMALL_SIZE = 24
    MEDIUM_SIZE = 28
    BIGGER_SIZE = 36
    #### FONT SIZE ###########################################################
    plt.rc('font', size=SMALL_SIZE)  # controls default text sizes
    plt.rc('axes', titlesize=MEDIUM_SIZE)  # fontsize of the axes title
    plt.rc('axes', labelsize=MEDIUM_SIZE)  # fontsize of the x and y labels
    plt.rc('xtick', labelsize=SMALL_SIZE)  # fontsize of the tick labels
    plt.rc('ytick', labelsize=MEDIUM_SIZE)  # fontsize of the tick labels
    plt.rc('legend', fontsize=SMALL_SIZE)  # legend fontsize
    plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

    # FONT #################################
    # print(matplotlib.get_cachedir())
    # fts = matplotlib.font_manager.findSystemFonts(fontpaths=None, fontext='ttf')
    # for ft in fts:
    #     print(ft)
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
    # plt.rcParams["text.usetex"] = False
    # plt.rcParams['text.latex.preamble'] = [r'\usepackage{amsmath}']
    # plt.rcParams['text.latex.preamble'] = r'\usepackage{{amsmath}}'

    for dataset in datasets_to_plot:
        graph_learner_results(active_learning_file, complete_dataset_learner, dataset)

    # plt.show()
    plt.tight_layout()
    plt.savefig('images/madden_3_col.pdf', dpi=200)

if __name__ == "__main__":
    graph_results()


