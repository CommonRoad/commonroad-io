Module Prediction
=================

The CommonRoad XML-specification provides three possibilities to describe the movement of dynamic obstacles over time: known behavior, unknown behavior bounded by sets, and unknown behavior described by probability distributions. Known behaviour and unknown behavior bounded by sets are described by the class :py:class:`TrajectoryPrediction` and :py:class:`SetBasedPrediction`. Unknown behavior described by probability distributions is not supported in version |version| and will be added in a future release. 

.. automodule:: commonroad.prediction.prediction

.. inheritance-diagram:: SetBasedPrediction TrajectoryPrediction
   :parts: 1

``Prediction`` class
--------------------
.. autoclass:: Prediction
   :members:

``TrajectoryPrediction`` class
------------------------------
.. autoclass:: TrajectoryPrediction
   :members:
   :inherited-members:

``SetBasedPrediction`` class
----------------------------
.. autoclass:: SetBasedPrediction
   :members:
   :inherited-members:

``Occupancy`` class
-------------------
.. autoclass:: Occupancy
   :members:

