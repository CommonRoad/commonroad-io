import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from commonroad.visualization.draw_dispatch_cr import draw_object, plottable_types
from typing import List, Union

def create_scenario_video(obj: Union[plottable_types, List[plottable_types]], time_begin: int, time_end: int,
                          file_path: str, plot_limits: Union[list,None], draw_params: Union[dict,None] = None,
                          fig_size: Union[list,None] = None, fps: int = 10, dpi=80):
    """
    Create scenario video in gif format given the path to the scenario xml
    :param filename: Name of the video to be saved
    :param scenario_path: path to the scenario xml used for creating the video gif
    :param add_only: true if you are only creating new videos and not updating the old ones
    :return:
    """
    assert time_begin < time_end, '<video/create_scenario_video> time_begin=%i needs to smaller than time_end=%i.' % (time_begin,time_end)

    if fig_size is None:
        fig_size = [15,8]

    fig = plt.figure(figsize=(fig_size[0], fig_size[1]))
    ln, = plt.plot([], [], animated=True)

    def update(frame=0):
        # plot frame
        plt.clf()
        plt.gca().set_aspect('equal')
        ax = plt.gca()
        draw_params = {
            'time_begin': time_begin + frame,
            'time_end': time_begin + min(frame_count,frame+duration)
        }
        draw_object(obj, ax=ax, draw_params=draw_params, plot_limits=plot_limits)
        # Set limits to assure that each frame has the same size

        return ln,

    frame_count = time_end - time_begin
    # Interval determines the duration of each frame
    interval = 1.0/fps

    # length of trajecotry steps
    duration = 1

    anim = FuncAnimation(fig, update, frames=frame_count,
                         init_func=update, blit=True, interval=interval)
    anim.save(file_path, dpi=dpi,
              writer='imagemagick')
    plt.close(plt.gcf())