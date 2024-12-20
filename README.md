# CommonRoad
[![PyPI pyversions](https://img.shields.io/pypi/pyversions/commonroad-io.svg)](https://pypi.python.org/pypi/commonroad-io/)  
![Linux](https://img.shields.io/badge/Linux-FCC624?style=for-the-badge&logo=linux&logoColor=black)
![Windows](https://img.shields.io/badge/Windows-0078D6?style=for-the-badge&logo=windows&logoColor=white)
![macOS](https://img.shields.io/badge/mac%20os-000000?style=for-the-badge&logo=macos&logoColor=F0F0F0)  
[![PyPI version fury.io](https://badge.fury.io/py/commonroad-io.svg)](https://pypi.python.org/pypi/commonroad-io/)
[![PyPI download month](https://img.shields.io/pypi/dm/commonroad-io.svg?label=PyPI%20downloads)](https://pypi.python.org/pypi/commonroad-io/) 
[![PyPI download week](https://img.shields.io/pypi/dw/commonroad-io.svg?label=PyPI%20downloads)](https://pypi.python.org/pypi/commonroad-io/)   
[![PyPI license](https://img.shields.io/pypi/l/commonroad-io.svg)](https://pypi.python.org/pypi/commonroad-io/)


Numerical experiments for motion planning of road vehicles require numerous ingredients: vehicle dynamics, 
a road network, static obstacles, dynamic obstacles and their movement over time, goal regions, a cost function, etc. 
Providing a description of the numerical experiment precise enough to reproduce it might require several pages of 
information. 
Thus, only key aspects are typically described in scientific publications, making it impossible to reproduce 
results - yet, reproducibility is an important asset of good science.

Composable benchmarks for motion planning on roads (CommonRoad) are proposed so that numerical experiments are fully 
defined by a unique ID; all required information to reconstruct the experiment can be found on [commonroad.in.tum.de](https://commonroad.in.tum.de/).
Each benchmark is composed of a [vehicle model](https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/blob/master/vehicleModels_commonRoad.pdf), 
a [cost function](https://gitlab.lrz.de/tum-cps/commonroad-cost-functions/blob/master/costFunctions_commonRoad.pdf), 
and a [scenario](https://commonroad.in.tum.de/scenarios/) (including goals and constraints). 
The scenarios are partly recorded from real traffic and partly hand-crafted to create dangerous situations. 
Solutions to the benchmarks can be uploaded and ranked on the CommonRoad Website.
Learn more about the scenario specification [here](https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf).

# commonroad-io

The commonroad-io package provides methods to read, write, and visualize CommonRoad scenarios and planning problems. Furthermore, it can be used as a framework for implementing motion planning algorithms to solve CommonRoad Benchmarks and is the basis for other tools of the CommonRoad Framework.
With commonroad-io, those solutions can be written to xml-files for uploading them on [commonroad.in.tum.de](https://commonroad.in.tum.de/).

This commonroad-io version is compatible with CommonRoad scenarios in version 2020a and supports reading 2018b scenarios.

The software is written in Python and tested on Linux for the Python 3.9, 3.10, 3.11, 3.12, and 3.13.


## Documentation

The full documentation of the API and introducing examples can be found under [https://cps.pages.gitlab.lrz.de/commonroad/commonroad-io/](https://cps.pages.gitlab.lrz.de/commonroad/commonroad-io/).

For getting started, we recommend our [tutorials](https://commonroad.in.tum.de/commonroad-io).

## Additional Tools
Based on commonroad-io, we have developed a list of tools supporting the development of motion-planning algorithms:

* [Drivability Checker](https://commonroad.in.tum.de/tools/drivability-checker)
* [CommonRoad-SUMO Interface](https://commonroad.in.tum.de/tools/sumo-interface)
* [Scenario Designer](https://commonroad.in.tum.de/tools/scenario-designer)
* [Vehicle Models](https://commonroad.in.tum.de/tools/model-cost-functions)
* [Dateset Converters](https://gitlab.lrz.de/tum-cps/dataset-converters)
* [Interactive Scenarios](https://gitlab.lrz.de/tum-cps/commonroad-interactive-scenarios)
* [Apollo Interface](https://gitlab.lrz.de/tum-cps/commonroad-apollo-interface)

## Requirements

The required dependencies for running commonroad-io are:

* numpy>=1.13
* scipy>=1.5.2
* shapely>=2.0.1
* matplotlib>=2.2.2
* lxml>=4.2.2
* networkx>=2.2
* Pillow>=7.0.0
* commonroad-vehicle-models>=2.0.0
* rtree>=0.8.3
* protobuf==3.20.1

## Installation

commonroad-io can be installed with::

	pip install commonroad-io

Alternatively, clone from our gitlab repository::

	git clone https://gitlab.lrz.de/tum-cps/commonroad_io.git

and add the folder commonroad-io to your Python environment.

## Changelog
A detailed overview about the changes in each version is provided in the [Changelog](https://github.com/CommonRoad/commonroad-io/blob/master/CHANGELOG.md).

## Authors
Contribution (in alphabetic order by last name): Yannick Ballnath, Behtarin Ferdousi, Luis Gressenbuch, Moritz Klischat, 
Markus Koschi, Sebastian Maierhofer, Stefanie Manzinger, Christina Miller, Christian Pek, Anna-Katharina Rettinger, 
Simon Sagmeister, Moritz Untersperger, Murat Üste, Xiao Wang

## Credits
We gratefully acknowledge partial financial support by

* DFG (German Research Foundation) Priority Program SPP 1835 Cooperative Interacting Automobiles
* BMW Group within the Car@TUM project
* German Federal Ministry of Economics and Technology through the research initiative Ko-HAF

## Citation
**If you use our code for research, please consider to cite our paper:**
```
@inproceedings{Althoff2017a,
	author = {Althoff, Matthias and Koschi, Markus and Manzinger, Stefanie},
	title = {CommonRoad: Composable benchmarks for motion planning on roads},
	booktitle = {Proc. of the IEEE Intelligent Vehicles Symposium},
	year = {2017},
	pages={719-726},
	abstract = {Numerical experiments for motion planning of road vehicles require numerous components: vehicle 
	            dynamics, a road network, static obstacles, dynamic obstacles and their movement over time, goal 
	            regions, a cost function, etc. Providing a description of the numerical experiment precise enough to 
	            reproduce it might require several pages of information. Thus, only key aspects are typically described 
	            in scientific publications, making it impossible to reproduce results—yet, re- producibility is an 
	            important asset of good science. Composable benchmarks for motion planning on roads (CommonRoad) are 
	            proposed so that numerical experiments are fully defined by a unique ID; all information required to 
	            reconstruct the experiment can be found on the CommonRoad website. Each benchmark is composed by a 
	            vehicle model, a cost function, and a scenario (including goals and constraints). The scenarios are 
	            partly recorded from real traffic and partly hand-crafted to create dangerous situations. We hope that 
	            CommonRoad saves researchers time since one does not have to search for realistic parameters of vehicle 
	            dynamics or realistic traffic situations, yet provides the freedom to compose a benchmark that fits 
	            one’s needs.},
}
```
