.. CommonRoad_io documentation master file, created by
   sphinx-quickstart on Tue Jul 10 09:17:31 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.


CommonRoad-io
=============

The commonroad-io package provides methods to read, write, and visualize CommonRoad scenarios and planning problems. Furthermore, it can be used as a framework for implementing motion planning algorithms to solve CommonRoad Benchmarks and is the basis for other tools of the CommonRoad Framework.
With commonroad-io, those solutions can be written to xml-files for uploading them on `commonroad.in.tum.de <https://commonroad.in.tum.de/>`__.

commonroad-io 2023.3 is compatible with CommonRoad scenarios of version 2020a and supports reading 2018b scenarios.

The software is written in Python and tested on Linux for the Python 3.8, 3.9, 3.10, and 3.11.

Documentation
=============

The full documentation of the API and introducing examples can be found under `commonroad.in.tum.de <https://commonroad-io.readthedocs.io/en/latest/>`__.

For getting started, we recommend our `tutorials <https://commonroad.in.tum.de/commonroad-io>`__.

Additional Tools
================
Based on commonroad-io, we have developed a list of tools for implementing motion-planning algorithms:

.. seealso::

    * `Drivability Checker <https://gitlab.lrz.de/tum-cps/commonroad-drivability-checker>`__
    * `CommonRoad-SUMO Interface <https://gitlab.lrz.de/tum-cps/commonroad-sumo-interface>`__
    * `CommonRoad Scenario Designer <https://gitlab.lrz.de/tum-cps/commonroad-scenario-designer>`__
    * `Vehicle Models <https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/tree/master/Python>`__
    * `Dateset Converters <https://gitlab.lrz.de/tum-cps/dataset-converters>`__
    * `Interactive Scenarios <https://gitlab.lrz.de/tum-cps/commonroad-interactive-scenarios>`__
    * `Apollo Interface <https://gitlab.lrz.de/tum-cps/commonroad-apollo-interface>`__

Requirements
============

The required dependencies for running commonroad-io are:

- numpy>=1.13
- scipy>=1.5.2
- shapely>=1.6.4
- matplotlib>=2.2.2
- lxml>=4.2.2
- networkx>=2.2
- Pillow>=7.0.0
- iso3166>=1.0.1
- rtree>=0.8.3
- protobuf==3.20.1

Installation
============

commonroad-io can be installed with::

	pip install commonroad-io

Alternatively, clone from our gitlab repository::

	git clone https://gitlab.lrz.de/tum-cps/commonroad_io.git

and add the folder commonroad-io to your Python environment.

Changelog
=========
Compared to version 2023.2, the following features have been added or changed:

Added
-----
- Type information for lanelet init function
- Dynamic obstacles can now store a history of their states
- Function to update the initial state of a dynamic obstacle while storing the previous state in the history
- Function to update behavior predictions of dynamic obstacles
- Function to find lanelet predecessors in range to lanelet network
- Function to compute all predecessor lanelets starting from a provided lanelet and merge them to a single lanelet for each route.
- Documentation for renderers (including video creation)
- Abstract interfaces for motion planner and prediction for usage in other tools
- New ExtendedPMState to support states with position, velocity, orientation, and acceleration
- Orientation property to PMState
- Hash and equality functions for area

Fixed
-----

- Function create_from_lanelet_network deletes references to removed lanelets
- Write environment time to XML in correct format
- Failing visualization of lanelets, stop lines, traffic signs, and traffic lights with z-coordinate
- Traffic lights now correctly change size in interactive matplotlib plots (only affected matplotlib>=3.7)
- Considering state attributes not part of dataclass definition in state to state conversion
- Enforce InitialState class for initial state property of dynamic obstacle
- Hash function of obstacle

Changed
-------

- Cleanup lanelet, traffic sign, and traffic light references in function create_from_lanelet_list by default
- Equality checks of scenario elements no longer emit a warning on inequality (except if the elements are of different types)

Removed
-------
- Duplicated initial_state property of dynamic obstacle

A detailed overview about the changes in each version is provided in the `Changelog <https://gitlab.lrz.de/tum-cps/commonroad_io/-/blob/master/CHANGELOG.md>`__.

Getting Started
===============

A tutorial on the main functionalities of the project is :ref:`available here<getting_started>`.


.. toctree::
   :maxdepth: 2
   :caption: Contents:

   user/index.rst
   api/index.rst


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Contact information
===================

:Website: `http://commonroad.in.tum.de <https://commonroad.in.tum.de/>`_
:Email: `commonroad@lists.lrz.de <commonroad@lists.lrz.de>`_
