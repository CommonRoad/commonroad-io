.. CommonRoad_io documentation master file, created by
   sphinx-quickstart on Tue Jul 10 09:17:31 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

=====================================
CommonRoad_io Documentation
=====================================

The CommonRoad_io package provides methods to read, write, and visualize CommonRoad scenarios and planning problems. Furthermore, it can be used as a framework for implementing motion planning algorithms to solve CommonRoad Benchmarks. Each benchmark is composed by a `vehicle model <https://commonroad.in.tum.de/documentation/vehicle_model_doc/>`_
, a `cost function <https://commonroad.in.tum.de/documentation/cost_func_doc/>`_
, and a `scenario <https://commonroad.in.tum.de/scenarios/>`_ (including goals and constraints). With CommonRoad_io those solutions can be written to xml-files for uploading them on `commonroad.in.tum.de <https://commonroad.in.tum.de/>`_ .


Installation
============

The package is written in Python 3.6 and tested on MacOs and Linux. The usage of the Anaconda_ Python distribution is recommended.

.. _Anaconda: http://www.anaconda.com/download/#download

The CommonRoad_io release 2019.1 is compatible only with `CommonRoad scenarios <https://gitlab.com/commonroad/commonroad.gitlab.io/tree/master>`_ from release 2018b.

The package is listed on `pypi.org/project/commonroad_io <https://pypi.org/project/commonroad_io/>`_ and can be installed from console via pip:

.. code-block:: console

	pip install commonroad-io

The required dependencies for running CommonRoad_io will be automatically installed when using the pip command above:

* numpy>=1.13
* shapely>=1.6.4
* matplotlib>=2.2.2
* networkx>=2.2

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
:Email: `commonroad-i06@in.tum.de <commonroad-i06@in.tum.de>`_
