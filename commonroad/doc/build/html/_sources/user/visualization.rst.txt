.. _visualization-manual:

====================
Visualization Manual
====================

.. _matplotlib: https://matplotlib.org

All objects of the scenario and planning problems can be visualized with the function :meth:`~commonroad.visualization.draw_dispatch_cr.draw_object` combined with matplotlib_. It is possible to pass either complete scenarios, single objects of the framework or lists of them to the drawing function. In case a list is passed, all objects therein have to be of the same type. Moreover, :meth:`~commonroad.visualization.draw_dispatch_cr.draw_object` provides customizable settings with the parameter ``draw_params`` and calls other specific drawing functions depending on the object type passed as an argument. For example ``draw_object(scenario)`` calls itself a specific function which is designed to draw a scenario. Often these functions itself call :meth:`~commonroad.visualization.draw_dispatch_cr.draw_object` for objects which belong to the original object. Nevertheless, the user only needs to call :meth:`~commonroad.visualization.draw_dispatch_cr.draw_object`.


``draw_params``
---------------

When settings of a plot should be changed with ``draw_params``, they have to be passed as a nested ``dict`` that refers to the hierarchy in which the objects are plotted. The complete structure of ``draw_params`` is given by the default parameters: ::
	
    {'time_begin': 0,
     'time_end': 50,
     'antialiased': True,
     'scenario':
            {'dynamic_obstacle':
                {'occupancy':
                    {'draw_occupancies': 0,
                     'shape': shape_parameters
                    },
                 'shape': shape_parameters,
                 'draw_shape': True,
                 'draw_icon': False,
                 'draw_bounding_box': True,
                 'show_label': False,
                 'trajectory_steps': 40,
                 'zorder': 20
                },
            'static_obstacle':
                {'shape': shape_parameters},
            'lanelet_network':
                {'lanelet':
                    {'left_bound_color': '#555555',
                     'right_bound_color': '#555555',
                     'center_bound_color': '#dddddd',
                     'draw_left_bound': True,
                     'draw_right_bound': True,
                     'draw_center_bound': True,
                     'draw_border_vertices': False,
                     'draw_start_and_direction': True,
                     'show_label': False,
                     'draw_linewidth': 0.5,
                     'fill_lanelet': True,
                     'facecolor': '#c7c7c7'
                    }
                }
            },
         'planning_problem_set':
            {'planning_problem':
                {'initial_state':
                    {'facecolor': '#000080',
                     'zorder': 25
                    }
                },
                 'goal_region':
                    {'draw_shape': True,
                     'shape': shape_parameters,
                     'lanelet':
                         {'left_bound_color': '#555555',
                          'right_bound_color': '#555555',
                          'center_bound_color': '#dddddd',
                          'draw_left_bound': True,
                          'draw_right_bound': True,
                          'draw_center_bound': True,
                          'draw_border_vertices': False,
                          'draw_start_and_direction': True,
                          'show_label': False,
                          'draw_linewidth': 0.5,
                          'fill_lanelet': True,
                          'facecolor': '#c7c7c7'
                         }
                    }
            }
     }

The defaults for ``shape_parameters`` are::

	shape_parameters = {'polygon':
			    	{'opacity': 0.2,
                       		 'facecolor': '#1d7eea',
	                         'edgecolor': '#0066cc',
	                         'zorder': 18
	                     	},
   	 		    'rectangle':
				{'opacity': 0.2,
				 'facecolor': '#1d7eea',
				 'edgecolor': '#0066cc',
				 'zorder': 18
				},
			    'circle':
				{'opacity': 0.2,
				 'facecolor': '#1d7eea',
				 'edgecolor': '#0066cc',
				 'zorder': 18
				}
			    }

Notice that specifying the the type of a shape in ``shape_parameters`` is optional and can be omitted. 


Passing custom ``draw_params``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In case no ``draw_params`` is passed to :meth:`~commonroad.visualization.draw_dispatch_cr.draw_object`, the default parameters are applied. To set a parameter manually, this needs to be done in accordance to the structure above. If for instance a complete scenario is plotted and the ``face_color`` of a dynamic obstacle should be set to black, this parameter can be specified by::

	draw_params = {'scenario': {'dynamic_obstacle': {'shape': {'facecolor':'#000000'}}}}
	draw_object(scenario,draw_params)


When the parameter of an object is extracted from ``draw_params``, the search starts at the lowest level of the ``dict``'s hierarchy. Therefore, it is sufficient to start with the specification on the lowest level of the dict, that unambiguously defines a parameter. Therefore, the expression above can be simplified to ::

	draw_params = {'dynamic_obstacle':{'shape':{'facecolor':'#000000'}}}

since ``dynamic_obstacle`` appears at no other point in the ``draw_params``' structure. On the other hand, in case you write::

	draw_params = {'shape':{'facecolor':'#000000'}}

all shapes in the plot will be drawn in black.

``plot_limits``
---------------

The drawn area of a scenario can be clipped by providing axes limits with :code:`plot_limits=[x_min, x_max, y_min, y_max]`. In cases where performance matters, this option should be preferred over setting axes limits with :code:`pyplot.get_gca().set_xlim`. The latter would only crop the shown area of the scenario after drawing the whole scenario.

Example plot with ``matplotlib``
--------------------------------

.. _matplotlib-API: https://matplotlib.org/api

The drawing function is used in combination with maplotlib. Therefore, every command from the matplotlib-API_ can be combined with ``draw_object``. A simple example for plotting scenario and the corresponding planning problem set with default parameters would be::
	
	import os
	import matplotlib.pyplot as plt
	from commonroad.common.file_reader import CommonRoadFileReader
	from commonroad.visualization.draw_dispatch_cr import draw_object
	filename = os.getcwd() + /scenarios/NGSIM/US101/USA_US101-2_1_T-1.xml'
	scenario, planning_problem_set = CommonRoadFileReader(filename).open()

	plt.style.use('classic')
	inch_in_cm = 2.54
	figsize = [20, 8]
	plot_limits = [-80, 80, -60, 30]
	plt.figure(figsize=(8,4.5))
	plt.gca().axis('equal')

	draw_object(scenario, plot_limits=plot_limits)
	draw_object(planning_problem_set, plot_limits=plot_limits)
	plt.show()

.. plot::
   :align: center
	
	#import os
	#import matplotlib.pyplot as plt
	#from commonroad.common.file_reader import CommonRoadFileReader
	#from commonroad.visualization.draw_dispatch_cr import draw_object
	#filename = os.getcwd() + '/../../../../../../scenarios/hand-crafted/DEU_Muc-1_1_T-1.xml'
	#scenario, planning_problem_set = CommonRoadFileReader(filename).open()

	#plt.style.use('classic')
	#inch_in_cm = 2.54
	#figsize = [20, 8]
	#plot_limits = [-80, 80, -60, 30]
	#plt.figure(figsize=(8,4.5))
	#plt.gca().axis('equal')

	#draw_object(scenario, plot_limits=plot_limits)
	#draw_object(planning_problem_set, plot_limits=plot_limits)
	#plt.tight_layout()
	#plt.show()
	
	import os
	import matplotlib.pyplot as plt
	from commonroad.common.file_reader import CommonRoadFileReader
	from commonroad.visualization.draw_dispatch_cr import draw_object
	filename = os.getcwd() + '/../../../../../../../scenarios/NGSIM/US101/USA_US101-2_1_T-1.xml'
	scenario, planning_problem_set = CommonRoadFileReader(filename).open()

	plt.style.use('classic')
	inch_in_cm = 2.54
	figsize = [20, 8]
	plot_limits = [-30, 120, -140, 20]
	plt.figure(figsize=(8,4.5))
	plt.gca().axis('equal')

	draw_object(scenario, draw_params={'time_end':20},plot_limits=plot_limits)
	draw_object(planning_problem_set, plot_limits=plot_limits)
	plt.tight_layout()
	plt.show()

.. _plot-helper:

Speed up plotting for real-time applications
--------------------------------------------

Plotting of a typical scenario can be too slow when using for real-time applications, where updated scenarios have have be redrawn at high rates. For those applications we provide the helper function ``redraw_obstacles()`` . Since plotting of the lanelet network requires most of the runtime, this function only updates obstacles of a scenario, while maintaining an initially plotted ``lanelet_network`` . Further speed improvements can be achieved by selecting a fast backend for matplotlib, like ``Qt5Agg`` or ``TkAgg`` .

Furthermore the number of plotted graphic elements should be minimized. These parameters help to improve run time considerably (ordered by impact)::

	draw_params = {'lanelet': {'draw_start_and_direction': False, 'draw_center_bound': False},
		       'dynamic_obstacle': {'trajectory_steps': 15}}

Additionally the plotted area should be restricted by using ``draw_object`` 's option ``plot_limits``. Effectively update rates of more than 20 frames/s are possible even for complex scenarios.

A minimal example would be::

	import matplotlib as mpl
	mpl.use('Qt5Agg') # sets the backend for matplotlib
	import mpl.pyplot as plt
	from commonroad.visualization.plot_helper import * 

	filename = os.getcwd() + /scenarios/NGSIM/US101/USA_US101-2_1_T-1.xml'
	scenario, planning_problem_set = CommonRoadFileReader(filename).open()

	set_non_blocking() # ensures interactive plotting is activated
	plt.style.use('classic')
	inch_in_cm = 2.54
	figsize = [30, 8]
	fig = plt.figure(figsize=(figsize[0] / inch_in_cm, figsize[1] / inch_in_cm))
	handles = {}  # collects handles of obstacles for fast updating of figures
	
	# inital plot including the lanelet network		
	draw_object(scenario, handles=handles)
	fig.canvas.draw()
	
	# loop where obstacle positions are modified
	for i in range(0,100):
		#...
		# modifying the scenario
		#...
		redraw_dynamic_obstacles(scenario, handles=handles, figure_handle=fig)
