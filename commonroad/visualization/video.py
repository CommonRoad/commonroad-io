import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from commonroad.visualization.draw_dispatch_cr import draw_object, plottable_types
from typing import List, Union


def create_scenario_video(obj: Union[plottable_types, List[plottable_types]], file_path: str, time_begin: int,
                          time_end: int, plotting_horizon=0, plot_limits: Union[list,None]=None, draw_params: Union[dict,None] = {},
                          fig_size: Union[list,None] = None, dt=0.1, fps: int = 25, dpi=120):
    """

    :param obj: object (list) to be plotted
    :param file_path: filename of generated video (ends on .mp4/.gif)
    :param time_begin: first time step of video
    :param time_end: last time step of video
    :param plotting_horizon: time steps of prediction plotted in each frame
    :param plot_limits: axis limits
    :param draw_params: draw_params (see draw_object doc)
    :param fig_size: size of the video
    :param fps: frames per second
    :param dpi: resolution of the video
    :return: None
    """
    assert time_begin < time_end, '<video/create_scenario_video> time_begin=%i needs to smaller than time_end=%i.' % (time_begin,time_end)

    if fig_size is None:
        fig_size = [15,8]
    fig = plt.figure(figsize=(fig_size[0], fig_size[1]))
    ln, = plt.plot([], [], animated=True)
    plt.gca().set_aspect('equal')

    def update(frame=0):
        # plot frame
        plt.clf()
        # plt.gca().set_aspect('equal')
        ax = plt.gca()
        draw_params.update({
            'time_begin': time_begin + frame,
            'time_end': time_begin + min(frame_count,frame+plotting_horizon)
        })
        draw_object(obj, ax=ax, draw_params=draw_params, plot_limits=plot_limits)
        # Set limits to assure that each frame has the same size

        return ln,

    frame_count = time_end - time_begin
    # Interval determines the duration of each frame
    interval = 1.0/fps/dt
    plt.ion()
    anim = FuncAnimation(fig, update, frames=frame_count,
                         init_func=update, blit=True, interval=interval)
    anim.save(file_path, dpi=dpi,
              writer='imagemagick')
    plt.close(plt.gcf())