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

import dgl
import grstapse
import torch


def main():
    scheduling_problem_inputs = grstapse.scheduling.loadProblemInputFromJson('')
    precedence_constraints = scheduling_problem_inputs.precedenceConstraints()
    pc_first = []
    pc_second = []
    for pc in precedence_constraints:
        pc_first.append(pc[0])
        pc_second.append(pc[1])

    mutex_constraints = scheduling_problem_inputs.mutexConstraints()
    # Filter out the precedence constraints
    mc_first = []
    mc_second = []
    for mc in mutex_constraints:
        if mc in precedence_constraints or [mc[1], mc[0]] in precedence_constraints:
            continue
        mc_first.append(mc[0])
        mc_second.append(mc[1])
    # Other direction
    for i in range(len(mc_first)):
        mc_first.append(mc_second[i])
        mc_second.append(mc_first[i])

    graph_data = {
        ('task', 'precedence', 'task'): (torch.tensor(pc_first), torch.tensor(pc_second)),
        ('task', 'mutex', 'task'): (torch.tensor(mc_first), torch.tensor(mc_second))
    }
    graph = dgl.heterograph(graph_data)
    # Set durations


if __name__ == '__main__':
    main()