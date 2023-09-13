Module Visualization
====================

This module implements functions to visualize all kind of objects from CommonRoad scenarios. Please refer to the :ref:`Visualization Manual <visualization-manual>` for detailed instructions and examples for using the API.

Drawing interface
--------------------

.. automodule:: commonroad.visualization.drawable
   :members:
   :undoc-members:
   :member-order: bysource

Drawing parameters
--------------------

.. automodule:: commonroad.visualization.draw_params
   :members:
   :undoc-members:
   :member-order: bysource

Renderers
--------------------

.. inheritance-diagram:: commonroad.visualization.renderer.IRenderer commonroad.visualization.mp_renderer.MPRenderer
   :parts: 1

``IRenderer`` provides the interface that all renderer implementations must adhere to.
Currently, the following renderers are implemented:

* ``MPRenderer``: A renderer using ``matplotlib`` as backend

``IRenderer`` class
^^^^^^^^^^^^^^^^^^^

.. automodule:: commonroad.visualization.renderer

.. autoclass:: IRenderer
   :members:
   :undoc-members:
   :member-order: bysource

``MPRenderer`` class
^^^^^^^^^^^^^^^^^^^^

.. automodule:: commonroad.visualization.mp_renderer

.. autoclass:: MPRenderer
   :members:
   :undoc-members:
   :member-order: bysource
