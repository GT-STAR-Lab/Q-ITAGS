# Q-ITAGS: Quality-Optimized Spatio-Temporal Heterogeneous Task Allocation with a Time Budget

## Description

This repository contains codes for **Q-ITAGS: Quality-Optimized Spatio-Temporal Heterogeneous Task Allocation with a Time Budget**. 

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

## Development

If you are interested in working on the development of this project please go to our wiki for information about setting
up your coding environment, our coding standards, and workflow.

# Citations

### [Graphically Recursive Simultaneous Task Allocation, Planning, and Scheduling]()

```
Messing, A., Neville, G., Chernova, S., Hutchinson, S., & Ravichandar, H. (2021). 
GRSTAPS: Graphically Recursive Simultaneous Task Allocation, Planning, and Scheduling. 
The International Journal of Robotics Research.
```

# Licensing

See [LICENSE](LICENSE)
