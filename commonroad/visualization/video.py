import matplotlib.pyplot as plt
from commonroad.visualization.scenario import MPRenderer
from commonroad.visualization.util import approximate_bounding_box_dyn_obstacles
from matplotlib.animation import FuncAnimation
from commonroad.visualization.draw_dispatch_cr import draw_object, \
    plottable_types
from typing import List, Union
import math


def create_scenario_video(obj_lists: List[plottable_types], file_path: str,
                          time_begin: int, time_end: int,
                          delta_time_steps: int = 1, plotting_horizon=0,
                          plot_limits: Union[list, None, str] = None,
                          draw_params: Union[dict, None] = {},
                          fig_size: Union[list, None] = None, dt=500, dpi=120):
    """
    Creates a video of one or multiple CommonRoad objects in mp4, gif,
    or avi format.

    :param obj: list of objects to be plotted. When plotting objects of the
    same type, use list of lists for speed-up.
    :param file_path: filename of generated video (ends on .mp4/.gif/.avi,
    default mp4, when nothing is specified)
    :param time_begin: first time step of video
    :param time_end: last time step of video
    :param delta_time_steps: plot every delta_time_steps time steps of scenario
    :param plotting_horizon: time steps of prediction plotted in each frame
    :param plot_limits: axis limits or 'auto' for limiting view to dynamic
    vehicles
    :param draw_params: draw_params (see draw_object doc)
    :param fig_size: size of the video
    :param dt: time step between frames in ms
    :param dpi: resolution of the video
    :return: None
    """
    assert time_begin < time_end, '<video/create_scenario_video> ' \
                                  'time_begin=%i needs to smaller than ' \
                                  'time_end=%i.' % (time_begin, time_end)

    if fig_size is None:
        fig_size = [15, 8]

    f, ax = plt.subplots(1, 1, figsize=fig_size)
    ax.set_aspect('equal')
    rnd = MPRenderer(draw_params=draw_params, plot_limits=plot_limits, ax=ax)

    def init_frame():
        rnd.draw_list(obj_lists, {
                'time_begin': time_begin,
                'time_end':   time_begin + delta_time_steps
        })
        rnd.render_static()
        artists = rnd.render_dynamic()
        if plot_limits is None:
            rnd.ax.autoscale()
        elif plot_limits == 'auto':
            limits = approximate_bounding_box_dyn_obstacles(obj_lists,
                                                            time_begin)
            if limits is not None:
                rnd.ax.xlim(limits[0][0] - 10, limits[0][1] + 10)
                rnd.ax.ylim(limits[1][0] - 10, limits[1][1] + 10)
            else:
                rnd.ax.autoscale()
        return artists

    def update(frame=0):
        rnd.clear()
        draw_params.update({
                'time_begin': time_begin + delta_time_steps * frame,
                'time_end':   time_begin + min(frame_count,
                                               delta_time_steps * frame +
                                               plotting_horizon)
        })
        rnd.remove_dynamic()
        rnd.draw_list(obj_lists, draw_params=draw_params)
        artists = rnd.render_dynamic()
        return artists

    # Min frame rate is 1 fps
    dt = max(1000.0, dt)
    frame_count = (time_end - time_begin) // delta_time_steps
    plt.ioff()
    # Interval determines the duration of each frame in ms
    anim = FuncAnimation(rnd.f, update, frames=frame_count,
                         init_func=init_frame, blit=False, interval=dt)

    if not any([file_path.endswith('.mp4'), file_path.endswith('.gif'),
                file_path.endswith('.avi')]):
        file_path += '.mp4'
    fps = int(math.ceil(1000.0 / dt))
    interval_seconds = dt / 1000.0
    anim.save(file_path, dpi=dpi, writer='ffmpeg', fps=fps,
              extra_args=["-g", "1", "-keyint_min", str(interval_seconds)])
