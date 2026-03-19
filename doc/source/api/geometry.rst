Module Geometry
=================

ObstacleShape and Occupancy
-------------------------------------

The class :class:`~commonroad.geometry.obstacle_shapes.obstacle_shape.ObstacleShape`
and its subclasses in the module :py:mod:`commonroad.geometry.obstacle_shapes`
are used to represent the general shape of an obstacle, while the class
:class:`~commonroad.geometry.occupancy.occupancy.Occupancy` and its subclasses in the module
:py:mod:`commonroad.geometry.occupancy` are used to represent the
occupancy of an obstacle in a certain state.
The method :meth:`~commonroad.geometry.obstacle_shapes.obstacle_shape.ObstacleShape.compute_occupancy_for_state`
computes the :class:`~commonroad.geometry.occupancy.occupancy.Occupancy` of an obstacle for a given state.
The following diagram visualizes both the :class:`~commonroad.geometry.obstacle_shapes.obstacle_shape.ObstacleShape`
and :class:`~commonroad.geometry.occupancy.occupancy.Occupancy` class hierarchies:

.. graphviz::

   digraph {
      rankdir=TB
      splines=ortho
      nodesep=0.3
      ranksep=1.0

      node [shape=box, style=filled, fillcolor=white, fontname=Helvetica, fontsize=10]
      edge [fontname=Helvetica, fontsize=9]

      // ObstacleShape hierarchy
      ObstacleShape [label="ObstacleShape", fillcolor="#E8F4F8"]
      RectObstacleShape [label="RectObstacleShape"]
      CircleObstacleShape [label="CircleObstacleShape"]
      PolygonObstacleShape [label="PolygonObstacleShape"]
      TruckShape [label="TruckShape"]
      SemiTrailerTruckShape [label="SemiTrailerTruckShape"]

      // Occupancy hierarchy
      Occupancy [label="Occupancy", fillcolor="#E8F4F8"]
      RectOccupancy [label="RectOccupancy"]
      CircleOccupancy [label="CircleOccupancy"]
      PolygonOccupancy [label="PolygonOccupancy"]
      OccupancyGroup [label="OccupancyGroup"]

      // ObstacleShape inheritance
      RectObstacleShape -> ObstacleShape [arrowhead=vee]
      CircleObstacleShape -> ObstacleShape [arrowhead=vee]
      PolygonObstacleShape -> ObstacleShape [arrowhead=vee]
      TruckShape -> ObstacleShape [arrowhead=vee]
      SemiTrailerTruckShape -> ObstacleShape [arrowhead=vee]

      // Occupancy inheritance
      RectOccupancy -> Occupancy [arrowhead=vee]
      CircleOccupancy -> Occupancy [arrowhead=vee]
      PolygonOccupancy -> Occupancy [arrowhead=vee]
      OccupancyGroup -> Occupancy [arrowhead=vee]

      // Relationships
      ObstacleShape -> Occupancy [style=dashed, arrowhead=open, label="compute_occupancy_for_state(state)"]

      newrank=true

        { rank=same; RectObstacleShape; CircleObstacleShape; PolygonObstacleShape; TruckShape; SemiTrailerTruckShape }
        { rank=same; ObstacleShape }
        { rank=same; Occupancy }
        { rank=same; RectOccupancy; CircleOccupancy; PolygonOccupancy; OccupancyGroup }

        // Chain one invisible edge per level transition to enforce ordering
        RectObstacleShape    ->  ObstacleShape [style=invis]
        RectObstacleShape    ->  RectOccupancy     [style=invis]
        Occupancy            ->  RectOccupancy       [style=invis]
   }

Module obstacle_shapes
----------------------

``ObstacleShape`` class
^^^^^^^^^^^^^^^^^^^^^^^
.. automodule:: commonroad.geometry.obstacle_shapes.obstacle_shape
.. autoclass:: ObstacleShape
   :members:

``RectObstacleShape`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. automodule:: commonroad.geometry.obstacle_shapes.rect_obstacle_shape
.. autoclass:: RectObstacleShape
   :members:

``CircleObstacleShape`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. automodule:: commonroad.geometry.obstacle_shapes.circle_obstacle_shape
.. autoclass:: CircleObstacleShape
   :members:

``PolygonObstacleShape`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. automodule:: commonroad.geometry.obstacle_shapes.polygon_obstacle_shape
.. autoclass:: PolygonObstacleShape
   :members:

``TruckShape`` class
^^^^^^^^^^^^^^^^^^^^
.. automodule:: commonroad.geometry.obstacle_shapes.truck_shape
.. autoclass:: TruckShape
   :members:
.. autoclass:: TruckDimensions
    :members:

``SemiTrailerTruckShape`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. automodule:: commonroad.geometry.obstacle_shapes.semi_trailer_truck_shape
.. autoclass:: SemiTrailerTruckShape
   :members:
.. autoclass:: TrailerDimensions
   :members:

Module occupancy
----------------

``Occupancy`` class
^^^^^^^^^^^^^^^^^^^
.. automodule:: commonroad.geometry.occupancy.occupancy
.. autoclass:: Occupancy
   :members:

``RectOccupancy`` class
^^^^^^^^^^^^^^^^^^^^^^^
.. automodule:: commonroad.geometry.occupancy.rect_occupancy
.. autoclass:: RectOccupancy
   :members:

``CircleOccupancy`` class
^^^^^^^^^^^^^^^^^^^^^^^^^
.. automodule:: commonroad.geometry.occupancy.circle_occupancy
.. autoclass:: CircleOccupancy
   :members:

``PolygonOccupancy`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^
.. automodule:: commonroad.geometry.occupancy.polygon_occupancy
.. autoclass:: PolygonOccupancy
   :members:

``OccupancyGroup`` class
^^^^^^^^^^^^^^^^^^^^^^^^
.. automodule:: commonroad.geometry.occupancy.occupancy_group
.. autoclass:: OccupancyGroup
   :members:

Transform
---------

.. automodule:: geometry.transform

.. autofunction:: translate_rotate

.. autofunction:: rotate_translate

.. autofunction:: rotation_translation_matrix

.. autofunction:: translation_rotation_matrix

.. autofunction:: to_homogeneous_coordinates

.. autofunction:: from_homogeneous_coordinates

Polyline Utility Functions
--------------------------

.. automodule:: geometry.polyline_util

.. autofunction:: compute_polyline_lengths

.. autofunction:: compute_total_polyline_length

.. autofunction:: compute_polyline_curvatures

.. autofunction:: compute_polyline_orientations

.. autofunction:: compute_polyline_initial_orientation

.. autofunction:: is_point_on_polyline

.. autofunction:: compute_polyline_intersections

.. autofunction:: is_polyline_self_intersection

.. autofunction:: compare_polylines_equality

.. autofunction:: resample_polyline_with_number

.. autofunction:: resample_polyline_with_distance

.. autofunction:: equalize_polyline_length

.. autofunction:: create_indices_mapping

.. autofunction:: concatenate_polylines
