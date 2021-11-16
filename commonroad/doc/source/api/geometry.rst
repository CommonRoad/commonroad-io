Module Geometry
=================

Shape
-----
.. automodule:: commonroad.geometry.shape

.. inheritance-diagram:: Shape Rectangle Circle Polygon ShapeGroup
   :parts: 1

``Shape`` class
^^^^^^^^^^^^^^^^^^^^
.. autoclass:: Shape
   :members:

``Rectangle`` class
^^^^^^^^^^^^^^^^^^^^
.. autoclass:: Rectangle
   :members:

``Circle`` class
^^^^^^^^^^^^^^^^^^^^
.. autoclass:: Circle
   :members:

``Polygon`` class
^^^^^^^^^^^^^^^^^^^^
.. autoclass:: Polygon
   :members:

``ShapeGroup`` class
^^^^^^^^^^^^^^^^^^^^
.. autoclass:: ShapeGroup
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
-------------

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

.. autofunction:: insert_vertices

.. autofunction:: create_indices_mapping

.. autofunction:: merge_polylines
