Module Scenario
===============

Scenario
--------

.. automodule:: commonroad.scenario.scenario

``Scenario`` class
^^^^^^^^^^^^^^^^^^
.. autoclass:: Scenario
   :members:
   :member-order: bysource

Road network
------------

.. automodule:: commonroad.scenario.lanelet

``LaneletNetwork`` class
^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: LaneletNetwork
   :undoc-members:   
   :members:
   :member-order: bysource


``Lanelet`` class
^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: Lanelet
   :undoc-members:   
   :members:
   :member-order: bysource


``LineMarking`` class
^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: LineMarking
   :members:
   :undoc-members:
   :member-order: bysource


Obstacles
---------

Different kinds of traffic participants are modeled as obstacles within the scenario. An obstacle is either static or dynamic. 

.. automodule:: commonroad.scenario.obstacle

.. inheritance-diagram:: StaticObstacle DynamicObstacle
   :parts: 1

``ObstacleRole`` class
^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: ObstacleRole
   :members:
   :undoc-members:
   :member-order: bysource

``ObstacleType`` class
^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: ObstacleType
   :members:
   :undoc-members:
   :member-order: bysource

``Obstacle`` class
^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: Obstacle
   :members:

``StaticObstacle`` class
^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: StaticObstacle
   :members:
   :inherited-members:

``DynamicObstacle`` class
^^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: DynamicObstacle
   :members:
   :inherited-members:


States and Trajectories
-----------------------

.. automodule:: commonroad.scenario.trajectory

``State`` class
^^^^^^^^^^^^^^^
.. autoclass:: State
   :members:
   :member-order: bysource

``Trajectory`` class
^^^^^^^^^^^^^^^^^^^^
.. autoclass:: Trajectory
   :members:
