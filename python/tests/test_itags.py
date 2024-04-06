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

import unittest

from grstapse import itags


class TestItags(unittest.TestCase):
    def test_itags_builder_options(self):
        builder_options = itags.ItagsBuilderOptions()

    def test_itags_problem_inputs(self):
        problem_inputs = itags.loadProblemInputsFromFile('data/itags_inputs.json')

    def test_itags_builder(self):
        builder_options = itags.ItagsBuilderOptions()
        builder = itags.ItagsBuilder(builder_options)
        problem_inputs = itags.loadProblemInputsFromFile('data/itags_inputs.json')
        solver = builder.build(problem_inputs)

    def test_itags(self):
        builder_options = itags.ItagsBuilderOptions()
        builder = itags.ItagsBuilder(builder_options)
        problem_inputs = itags.loadProblemInputsFromFile('data/itags_inputs.json')
        solver = builder.build(problem_inputs)
        results = solver.search()
        results.writeToFile('output.json', problem_inputs)


if __name__ == '__main__':
    unittest.main()