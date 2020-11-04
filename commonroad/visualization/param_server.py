import json
import logging


def create_default_draw_params() -> dict:
    basic_shape_parameters_static = {
            'opacity':   1.0,
            'facecolor': '#d95558',
            'edgecolor': '#831d20',
            'linewidth': 0.5,
            'zorder':    20}

    basic_shape_parameters_dynamic = {
            'opacity':   1.0,
            'facecolor': '#1d7eea',
            'edgecolor': '#0066cc',
            'linewidth': 0.5,
            'zorder':    20}

    draw_params = {
            'dynamic_obstacle': {
                    'draw_shape':        True,
                    'draw_icon':         False,
                    'draw_bounding_box': True,
                    'show_label':        False,  # show id of obstacle
                    'zorder':            20,
                    'draw_signals':      True,  # draw signal states
                    'signal_radius':     0.5,  # size of a signal states
                    'indicator_color':   '#ebc200',  # turn light
                    'braking_color':     'red',  # braking light
                    'blue_lights_color': 'blue',
                    'horn_color':        'red',
                    # horn is visualized as red center dot
                    'initial_state':     {'draw_initial_state': False,
                                          # visualize initial state
                                          # by arrow
                                          # proportional to velocity
                                          'scale_factor':       0.3,
                                          # length of arrow in m per m/s
                                          'kwargs':             {
                                                  'linewidth':            1.5,
                                                  'length_includes_head': True,
                                                  'edgecolor':
                                                      'black',
                                                  'facecolor':
                                                      'black', },
                                          # further parameters for
                                          # FancyArrow
                                          },
                    'shape':             {
                            'polygon':   basic_shape_parameters_dynamic,
                            'rectangle': basic_shape_parameters_dynamic,
                            'circle':    basic_shape_parameters_dynamic},
                    'trajectory':        {
                            'draw_trajectory': True,
                            'facecolor':       '#000000',
                            'draw_continuous': False,
                            'unique_colors':   False,
                            'line_width':      0.17,
                            'z_order':         24}},
            'lanelet_network':  {'kwargs_traffic_light_signs': {},
                                 # further properties for AnnotationBox of
                                 # traffic
                                 # signs or lights, see #
                                 # https://matplotlib.org/3.1.0/gallery
                                 # /text_labels_and_annotations/demo_annotation_box.html
                                 },
            'traffic_light':    {
                    'draw_traffic_lights': True,
                    'red_color':           'red',
                    'yellow_color':        '#feb609',
                    'green_color':         '#00aa16',
                    'red_yellow_color':    '#fe4009ff',
                    'show_label':          False,
                    'scale_factor':        0.25,
                    'zorder':              30},
            'traffic_sign':     {
                    'draw_traffic_signs': False,
                    'show_traffic_signs': 'all',
                    # 'all' or list of TrafficSignIDs
                    'speed_limit_unit':   'auto',  # 'mph', 'kmh', 'ms', 'auto'
                    'show_label':         False,
                    'scale_factor':       0.25,
                    'zorder':             30},
            'intersection':     {
                    'draw_intersections':        False,
                    'draw_incoming_lanelets':    True,
                    'incoming_lanelets_color':   '#3ecbcf',
                    'draw_crossings':            True,
                    'crossings_color':           '#b62a55',
                    'draw_successors':           True,
                    'successors_left_color':     '#ff00ff',
                    'successors_straight_color': 'blue',
                    'successors_right_color':    '#ccff00',
                    'show_label':                False,
                    # show incoming id and incoming left
            },
            'lanelet':          {
                    'left_bound_color':         '#555555',
                    'right_bound_color':        '#555555',
                    'center_bound_color':       '#dddddd',
                    'unique_colors':            False,
                    # colorizes center_vertices and labels of
                    # each lanelet differently
                    'draw_stop_line':           True,
                    'stop_line_color':          '#ffffff',
                    'draw_line_markings':       True,
                    'draw_left_bound':          True,
                    'draw_right_bound':         True,
                    'draw_center_bound':        True,
                    'draw_border_vertices':     False,
                    'draw_start_and_direction': True,
                    'show_label':               False,  # show lanelet_id
                    'draw_linewidth':           0.5,
                    'fill_lanelet':             True,
                    'facecolor':                '#c7c7c7'},
            'occupancy':        {'draw_occupancies': 0,  # -1= never,
                                 # 0= if prediction of vehicle
                                 # is set-based, 1=always
                                 'shape':            {
                                         'polygon':   {
                                                 'opacity':   0.2,
                                                 'facecolor': '#1d7eea',
                                                 'edgecolor': '#0066cc',
                                                 'linewidth': 0.5,
                                                 'zorder':    18, },
                                         'rectangle': {
                                                 'opacity':   0.2,
                                                 'facecolor': '#1d7eea',
                                                 'edgecolor': '#0066cc',
                                                 'linewidth': 0.5,
                                                 'zorder':    18, },
                                         'circle':    {
                                                 'opacity':   0.2,
                                                 'facecolor': '#1d7eea',
                                                 'edgecolor': '#0066cc',
                                                 'linewidth': 0.5,
                                                 'zorder':    18, }}, },
            'shape':            {
                    'polygon':   basic_shape_parameters_static,
                    'rectangle': basic_shape_parameters_static,
                    'circle':    basic_shape_parameters_static, }}
    # ensure that parameters are also available on higher levels
    draw_params['shape'].update(basic_shape_parameters_static)

    return draw_params


class ParamServer:
    def __init__(self, data=None):
        self.data = data or {}
        # with open('.')
        self.defaults = create_default_draw_params()

    @staticmethod
    def _resolve_key(map, key):
        val = None
        for k in key:
            val = map[k]
            if val is None:
                break
        if val is None and len(key) > 0:
            return ParamServer._resolve_key(map, key[1:])
        else:
            return val

    def __getitem__(self, item):
        if not isinstance(item, tuple):
            item = tuple(item)

        val = ParamServer._resolve_key(self.defaults, item)
        if val is None:
            logging.warning('Value for key {} not found!'.format(item))
        return val  # if success:  #     return val  # else:  #     # Value
        # was not found  #     val, success = ParamServer._resolve_key(  #  #
        # self.defaults, item)  #     if success:  #         logging.warning(
        # 'Using default for {}'.format(item))  #         return val

    # @staticmethod  # def from_json(fname):  #     data =
