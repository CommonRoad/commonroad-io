Module Scenario
===============


Scenario
--------

.. automodule:: commonroad.scenario.scenario

``Scenario`` class
^^^^^^^^^^^^^^^^^^
.. autoclass:: Scenario
   :undoc-members:
   :members:
   :member-order: bysource


``ScenarioID`` class
^^^^^^^^^^^^^^^^^^^^
.. autoclass:: ScenarioID
   :undoc-members:
   :members:
   :member-order: bysource


``Tag`` class
^^^^^^^^^^^^^
.. autoclass:: Tag
  :undoc-members:
  :members:
  :member-order: bysource


``Time`` class
^^^^^^^^^^^^^^
.. autoclass:: Time
  :undoc-members:
  :members:
  :member-order: bysource


``Environment`` class
^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: Environment
  :undoc-members:
  :members:
  :member-order: bysource


``GeoTransformation`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: GeoTransformation
  :undoc-members:
  :members:
  :member-order: bysource


``Location`` class
^^^^^^^^^^^^^^^^^^
.. autoclass:: Location
  :undoc-members:
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
^^^^^^^^^^^^^^^^^
.. autoclass:: Lanelet
   :undoc-members:
   :members:
   :member-order: bysource


``LaneletType`` class
^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: LaneletType
   :members:
   :undoc-members:
   :member-order: bysource


``RoadUser`` class
^^^^^^^^^^^^^^^^^^
.. autoclass:: RoadUser
   :members:
   :undoc-members:
   :member-order: bysource


Traffic Sign
------------

.. automodule:: commonroad.scenario.traffic_sign

``SupportedTrafficSignCountry`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: SupportedTrafficSignCountry
   :members:
   :undoc-members:
   :member-order: bysource


``TrafficSignIDZamunda`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: TrafficSignIDZamunda
   :members:
   :undoc-members:
   :member-order: bysource


``TrafficSignIDGermany`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: TrafficSignIDGermany
   :members:
   :undoc-members:
   :member-order: bysource


``TrafficSignIDUsa`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: TrafficSignIDUsa
   :members:
   :undoc-members:
   :member-order: bysource


``TrafficSignIDChina`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: TrafficSignIDChina
   :members:
   :undoc-members:
   :member-order: bysource


``TrafficSignIDSpain`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: TrafficSignIDSpain
   :members:
   :undoc-members:
   :member-order: bysource


``TrafficSignIDRussia`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: TrafficSignIDRussia
   :members:
   :undoc-members:
   :member-order: bysource


Traffic Light
-------------

``TrafficLight`` class
^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: TrafficLight
   :members:
   :undoc-members:
   :member-order: bysource

``TrafficLightDirection`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: TrafficLightDirection
   :members:
   :undoc-members:
   :member-order: bysource


``TrafficLightState`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: TrafficLightState
   :members:
   :undoc-members:
   :member-order: bysource


``TrafficSignElement`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: TrafficSignElement
   :members:
   :undoc-members:
   :member-order: bysource


``TrafficSign`` class
^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: TrafficSign
   :members:
   :undoc-members:
   :member-order: bysource


``TrafficLightCycleElement`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: TrafficLightCycleElement
   :members:
   :undoc-members:
   :member-order: bysource


Traffic Sign Interpreter
------------------------

.. automodule:: commonroad.scenario.traffic_sign_interpreter

``TrafficSigInterpreter`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: TrafficSigInterpreter
   :members:
   :undoc-members:
   :member-order: bysource



.. automodule:: commonroad.scenario.intersection

``IntersectionIncomingElement`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: IntersectionIncomingElement
   :members:
   :undoc-members:
   :member-order: bysource


Intersection
------------

``Intersection`` class
^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: Intersection
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

``SignalState`` class
^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: SignalState
 :members:
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
