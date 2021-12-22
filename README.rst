CommonRoad
============

Numerical experiments for motion planning of road vehicles require numerous ingredients: vehicle dynamics, a road network, static obstacles, dynamic obstacles and their movement over time, goal regions, a cost function, etc. Providing a description of the numerical experiment precise enough to reproduce it might require several pages of information. Thus, only key aspects are typically described in scientific publications, making it impossible to reproduce results - yet, reproducibility is an important asset of good science.

Composable benchmarks for motion planning on roads (CommonRoad) are proposed so that numerical experiments are fully defined by a unique ID; all required information to reconstruct the experiment can be found on `commonroad.in.tum.de <https://commonroad.in.tum.de/>`_
. Each benchmark is composed of a `vehicle model <https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/blob/master/vehicleModels_commonRoad.pdf>`__, a `cost function <https://gitlab.lrz.de/tum-cps/commonroad-cost-functions/blob/master/costFunctions_commonRoad.pdf>`__, and a `scenario <https://commonroad.in.tum.de/scenarios/>`__ (including goals and constraints). The scenarios are partly recorded from real traffic and partly hand-crafted to create dangerous situations. Solutions to the benchmarks can be uploaded and ranked on the CommonRoad Website.
Learn more about the scenario specification `here <https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf>`__.

CommonRoad_io
=============

The CommonRoad_io package provides methods to read, write, and visualize CommonRoad scenarios and planning problems. Furthermore, it can be used as a framework for implementing motion planning algorithms to solve CommonRoad Benchmarks and is the basis for other tools of the CommonRoad Framework.
With CommonRoad_io, those solutions can be written to xml-files for uploading them on `commonroad.in.tum.de <https://commonroad.in.tum.de/>`__.

CommonRoad_io 2021.2 is compatible with CommonRoad scenario in version 2020a and supports reading 2018b scenarios.

The software is written in Python and tested on Linux for the Python 3.6, 3.7, 3.8, 3.9, and 3.10. The usage of the Anaconda_ Python distribution is strongly recommended.

.. _Anaconda: http://www.anaconda.com/download/#download

Documentation
=============

The full documentation of the API and introducing examples can be found under `commonroad.in.tum.de <https://commonroad-io.readthedocs.io/en/latest/>`__.

For getting started, we recommend our `tutorials <https://commonroad.in.tum.de/commonroad-io>`__.

Additional Tools
================
Based on CommonRoad_io, we have developed a list of tools for implementing motion-planning algorithms:

* `Drivability Checker <https://gitlab.lrz.de/tum-cps/commonroad-drivability-checker>`__
* `CommonRoad-SUMO Interface <https://gitlab.lrz.de/tum-cps/commonroad-sumo-interface>`__
* `OpenDRIVE to Lanelet converter <https://pypi.org/project/opendrive2lanelet>`__
* `Vehicle Models <https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/tree/master/Python>`__

Requirements
============

The required dependencies for running CommonRoad_io are:

* numpy>=1.13
* scipy>=1.5.2
* shapely>=1.6.4
* matplotlib>=2.2.2
* lxml>=4.2.2
* networkx>=2.2
* Pillow>=7.0.0
* commonroad-vehicle-models>=2.0.0
* rtree>=0.8.3

Installation
============

CommonRoad_io can be installed with::

	pip install commonroad-io

Alternatively, clone from our gitlab repository::

	git clone https://gitlab.lrz.de/tum-cps/commonroad_io.git

and add the folder commonroad_io to your Python environment.

Changelog
============
Compared to version 2021.3, the following features have been added or changed:

* Polyline utility functions, e.g., resampling, path length, orientation, curvature, intersection
* `__eq__` and `__hash__` functions for LaneletNetwork and related classes (e.g., traffic sign, traffic light, stop
  line, etc.)
* Compatibility for Shapely 2.0
* License switched to BSD-3
* New traffic signs for Germany
* Date in solution file now stored in the dateTime format (`%Y-%m-%dT%H:%M:%S`)

A detailed overview about the changes in each version is provided in the Changelog.
