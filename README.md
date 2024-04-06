# Graphically Recursive Simultaneous Task Allocation, Planning, Scheduling, and Execution

[![ci](https://github.com/amessing/grstapse/actions/workflows/ci.yml/badge.svg)](https://github.com/amessing/grstapse/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/amessing/grstapse/branch/master/graph/badge.svg)](https://codecov.io/gh/amessing/grstapse)

## Description

This repository contains the various components of GRSTAPS+E. These include a refactored version of **Forward Chaining
Partial-Order Planning (FCPOP)**, **Incremental Task Allocation Graph Search (Itags)**, and **Graphically Recursive Task
Allocation, Planning, and Scheduling (GRSTAPS)**. Additionally, it contains the original implementations of **Dynamic
Incremental Task Allocation Graph Search (DITAGS)**. The refactoring uses the same algorithms from the respective papers
except for the algorithm for scheduling. In the Dynamic Incremental Task Allocation Graph Search paper, we describe the
change to the algorihtm for scheduling and demonstrate the improved efficiency of the refactored version to the
original. Furthermore, we believe the refactored version improve the code quality, making development easier, and also
is easier to extend for future research.

## Dependencies

**Note: We list all the dependencies down below to recognize the libraries that we use; however, all development should
be done in the docker container. Any issues from trying to create a local environment will not be a priority.**

### apt

- libboost-all-dev
- libeigen3-dev
- libgeos-dev
- libompl-dev
- libyaml-cpp-dev

### pip

Currently none

### mirrored public repositories

- [benchmark](https://github.com/amessing/benchmark) ([original](https://github.com/google/benchmark))
- [cli11](https://github.com/amessing/cli11) ([original]())
- [fmt](https://github.com/amessing/fmt) ([original](https://github.com/fmtlib/fmt))
- [googletest](https://github.com/amessing/googletest) ([original](https://github.com/google/googletest))
- [json](https://github.com/amessing/json) ([original](https://github.com/nlohmann/json))
- [magic enum](https://github.com/amessing/magic_enum) ([original](https://github.com/Neargye/magic_enum))
- [robin-hood-hashing](https://github.com/amessing/robin-hood-hashing) ([original](https://github.com/martinus/robin-hood-hashing))
- [spdlog](https://github.com/amessing/spdlog) ([original](https://github.com/gabime/spdlog))

### other

#### Gurobi

We use [Gurobi](https://www.gurobi.com/) to solve Mixed Integer Linear Programming problems. In order to use this with
the docker, you will need to get a Web
License ([click here](https://www.gurobi.com/academia/academic-program-and-licenses/)) and then put the
associated ```gurobi.lic``` file in ```docker/gurobi```. The `run.sh` script in the docker folder will mount that file
to the docker container so that it can use gurobi. The license file is in the .gitignore and should under ___NO___
circumstance become part of the repository (simply do not change that line and this shouldn't ever be something to worry
about). You are responsible for your own gurobi license.

## Usage

This library is used to run experiments for academic papers. Instructions to run the experiments for those papers are
listed below:

### WAFR 2022

TODO

## Development

If you are interested in working on the development of this project please go to our wiki for information about setting
up your coding environment, our coding standards, and workflow.

# Citations

### [Forward Chaining Hierarchical Partial-Order Planning](http://robotics.cs.rutgers.edu/wafr2020/wp-content/uploads/sites/7/2020/05/WAFR_2020_FV_43.pdf)

```
Messing, A., & Hutchinson, S. (2020, June). Forward chaining hierarchical partial-order planning. 
In International Workshop on the Algorithmic Foundations of Robotics (pp. 364-380). Springer, Cham.
```

### [Incremental Task Allocation Graph Search]()

```
Neville, G., Messing, A., Ravichandar, H., Hutchinson, S., & Chernova, S. (2021, August). 
An interleaved approach to trait-based task allocation and scheduling. In 2021 IEEE/RSJ 
International Conference on Intelligent Robots and Systems (IROS) (pp. 1507-1514). IEEE.
```

### [Graphically Recursive Simultaneous Task Allocation, Planning, and Scheduling]()

```
Messing, A., Neville, G., Chernova, S., Hutchinson, S., & Ravichandar, H. (2021). 
GRSTAPS: Graphically Recursive Simultaneous Task Allocation, Planning, and Scheduling. 
The International Journal of Robotics Research.
```

# Licensing

See [LICENSE](LICENSE)