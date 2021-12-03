.. _visualization-manual:

====================
Visualization Manual
====================

.. _matplotlib: https://matplotlib.org
.. _`visitor pattern`: https://en.wikipedia.org/wiki/Visitor_pattern
.. _matplotlib-API: https://matplotlib.org/api

For the visualization of CommonRoad, we use the `visitor pattern`_. A visitor object, which we call *renderer* subsequently, has first to be instantiated. Afterward, the drawing function :meth:`~commonroad.visualization.drawable.IDrawable.draw` of the :class:`~commonroad.visualization.drawable.IDrawable` interface has to be called on all objects that should be visualized. All objects of the scenario and planning problems implement this function and can be individually drawn. Finally when all objects were drawn, :meth:`~commonroad.visualization.mp_renderer.MPRenderer.render` is called on the renderer to show the objects. Currently, matplotlib_ is the only supported rendering engine in CommonRoad. It is implemented in the :class:`~commonroad.visualization.mp_renderer.MPRenderer`-class.

Creating plots in CommonRoad
----------------------------
The drawing function is used in combination with matplotlib_. Therefore, the visualization of CommonRoad objects can be combined with standard matplotlib drawing functions. A simple example for plotting scenario and the corresponding planning problem set with default parameters would be::

    from commonroad.common.file_reader import CommonRoadFileReader
    from commonroad.visualization.mp_renderer import MPRenderer
    filename = 'path/to/scenario/USA_US101-2_1_T-1.xml'
    scenario, planning_problem_set = CommonRoadFileReader(filename).open()

    rnd = MPRenderer(plot_limits=[-30, 120, -140, 20], figsize=(8,4.5))
    scenario.draw(rnd)
    planning_problem_set.draw(rnd)
    rnd.render()

.. plot::
   :align: center


    import os
    import matplotlib.pyplot as plt
    from commonroad.common.file_reader import CommonRoadFileReader
    from commonroad.visualization.mp_renderer import MPRenderer
    filename = os.getcwd() + '/../../../tests/test_scenarios/USA_US101-4_1_T-1.xml'
    scenario, planning_problem_set = CommonRoadFileReader(filename).open()

    rnd = MPRenderer(plot_limits=[-30, 120, -140, 20], figsize=(8,4.5))
    scenario.draw(rnd)
    planning_problem_set.draw(rnd)
    rnd.render()



Styling options
---------------

The plotting style of the visualization module can be adjusted via two mechanisms:

    1. Setting a set of default parameters in the constructor of :class:`~commonroad.visualization.mp_renderer.MPRenderer`, and
    2. Overriding the default parameters by setting parameters on a per-object basis, when calling individual :meth:`~commonroad.visualization.drawable.IDrawable.draw` functions.

The full set of available parameters can be written to a file in JSON format by calling :meth:`~commonroad.visualization.param_server.write_default_params`. Parameters can be passed as nested dictionary, where the levels correspond to the hierarchy of elements in a compound object. The resolution of plotting properties starts at the most specific (deepest) level. If the property is not found, the topmost level is removed and resolution is restarted. For example, the default vehicle shape's color is specified on the path ``draw_params["dynamic_obstacle"]["vehicle_shape"]["occupancy"]["shape"]["rectangle"]["facecolor"]``. When a dynamic obstacle is drawn as part of a scenario, the resolution process is started at ``draw_params["scenario"]["dynamic_obstacle"]["vehicle_shape"]...["facecolor"]``. As the key is not found in the default parameter set, resolution is retried with ``draw_params["dynamic_obstacle"]["vehicle_shape"]...["facecolor"]`` and is successful.

Example
"""""""
.. _`matplotlib colors`: https://matplotlib.org/stable/tutorials/colors/colors.html

We want to draw dynamic obstacles with a rectangular shape in a scenario with a green and individual dynamic obstacles with a rectangular shape in yellow. This can be achieved by passing the following drawing parameters: ::

    {
        "dynamic_obstacle": {
            "occupancy": {
                "shape": {
                    "rectangle": {
                        "facecolor": "yellow",
                    },
                }
            }
        },
        "scenario": {
            "dynamic_obstacle": {
                "occupancy": {
                    "shape": {
                        "rectangle": {
                            "facecolor": "green",
                        },
                    }
                }
            }
        }
    }

Note, that colors are specified as `matplotlib colors`_.

``plot_limits``
---------------

The drawn area of a scenario can be clipped by providing axes limits with :code:`plot_limits=[x_min, x_max, y_min, y_max]` to the renderer. In cases where performance matters, this option should be preferred over setting axes limits with :code:`pyplot.get_gca().set_xlim`. The latter would only crop the shown area of the scenario after drawing the whole scenario.
