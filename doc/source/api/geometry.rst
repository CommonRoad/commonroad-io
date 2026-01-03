Module Geometry
=================

ObstacleShape
-------------
.. inheritance-diagram:: commonroad.geometry.obstacle_shapes.obstacle_shape.ObstacleShape commonroad.geometry.obstacle_shapes.rect_obstacle_shape.RectObstacleShape commonroad.geometry.obstacle_shapes.circle_obstacle_shape.CircleObstacleShape commonroad.geometry.obstacle_shapes.polygon_obstacle_shape.PolygonObstacleShape commonroad.geometry.obstacle_shapes.truck_shape.TruckShape commonroad.geometry.obstacle_shapes.semi_trailer_truck_shape.SemiTrailerTruckShape
   :parts: 1

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

``SemiTrailerTruckShape`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. automodule:: commonroad.geometry.obstacle_shapes.semi_trailer_truck_shape
.. autoclass:: SemiTrailerTruckShape
   :members:

Occupancy
---------
.. inheritance-diagram:: commonroad.geometry.occupancy.occupancy.Occupancy commonroad.geometry.occupancy.rect_occupancy.RectOccupancy commonroad.geometry.occupancy.circle_occupancy.CircleOccupancy commonroad.geometry.occupancy.polygon_occupancy.PolygonOccupancy commonroad.geometry.occupancy.occupancy_group.OccupancyGroup
   :parts: 1

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
