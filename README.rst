CommonRoad
============

Numerical experiments for motion planning of road vehicles require numerous ingredients: vehicle dynamics, a road network, static obstacles,dynamic obstacles and their movement over time, goal regions, a cost function, etc. Providing a description of the numerical experiment precise enough to reproduce it might require several pages of information. Thus, only key aspects are typically described in scientific publications, making it impossible to reproduce results - yet, reproducibility is an important asset of good science.

Composable benchmarks for motion planning on roads (CommonRoad) are proposed so that numerical experiments are fully defined by a unique ID; all required information to reconstruct the experiment can be found on `commonroad.in.tum.de <https://commonroad.in.tum.de/>`_
. Each benchmark is composed by a `vehicle model <https://commonroad.in.tum.de/documentation/vehicle_model_doc/>`_
, a `cost function <https://commonroad.in.tum.de/documentation/cost_func_doc/>`_
, and a `scenario <https://commonroad.in.tum.de/scenarios/>`_ (including goals and constraints). The scenarios are partly recorded from real traffic and partly hand-crafted to create dangerous situations. Solutions to the benchmarks can be uploaded and ranked on the CommonRoad Website.

CommonRoad_io
=============

The CommonRoad_io package provides methods to read, write, and visualize CommonRoad scenarios and planning problems. Furthermore, it can be used as a framework for implementing motion planning algorithms to solve CommonRoad Benchmarks and is the basis for other tool of the CommonRoad Framework.
With CommonRoad_io, those solutions can be written to xml-files for uploading them on `commonroad.in.tum.de <https://commonroad.in.tum.de/>`_ .

The software is written in Python 3.6 and tested on MacOs and Linux. The usage of the Anaconda_ Python distribution is strongly recommended.

.. _Anaconda: http://www.anaconda.com/download/#download

Documentation
=============

The full documentation of the API and introducing examples can be found under `commonroad-io.readthedocs.io <https://commonroad-io.readthedocs.io>`_

Requirements
============

The required dependencies for running CommonRoad_io are:

* numpy>=1.13
* shapely>=1.6.4
* matplotlib>=2.2.2
* networkx>=2.2

Installation
============

CommonRoad_io can be installed with::

	pip install commonroad-io
