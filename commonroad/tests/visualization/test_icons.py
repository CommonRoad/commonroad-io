import unittest
from commonroad.visualization import icons
from commonroad.scenario.obstacle import ObstacleType

# This needs to be imported. I don't know why but otherwise,
# the mpl patches can not be imported.
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np


__author__ = "Simon Sagmeister"
__copyright__ = "TUM Cyber-Physical Systems Group"
__version__ = "2020.4"
__email__ = "simon.sagmeister@tum.de"
__status__ = "Development"


class TestIcons(unittest.TestCase):
    @staticmethod
    def _get_draw_funcs():
        return set(icons._obstacle_icon_assignment().values())

    def test_icons_for_execution(self):
        """Test if the icons return patches without raising an exception."""

        for draw_func in self._get_draw_funcs():
            patch_list = draw_func(
                pos_x=0,
                pos_y=0,
                orientation=0,
            )
            for patch in patch_list:
                assert isinstance(patch, mpl.patches.Patch)

    def test_show_icons(self):
        """Draw the icons that are generated from individual draw funcs."""
        fig = plt.figure()
        ax = fig.gca()
        draw_funcs = self._get_draw_funcs()
        for counter, draw_func in enumerate(draw_funcs):
            patch_list = draw_func(
                pos_x=10 * counter,
                pos_y=0,
                orientation=np.pi / 4,
            )
            for patch in patch_list:
                ax.add_patch(patch)
        ax.set_xlim(-10, len(draw_funcs) * 10)
        ax.set_ylim(-20, 20)
        ax.axis("equal")
        plt.show()

    def test_get_obstacle_icon_patch(self):
        """Draw the obstacle icons in the norm square for all supported vehicle types."""
        fig = plt.figure()
        ax = fig.gca()
        vehicle_types = icons.supported_icons()
        for counter, vehicle_type in enumerate(vehicle_types):

            # Draw the bounding box
            ax.add_patch(
                mpl.patches.Rectangle(
                    (110 * counter - 50, -50), 100, 100, color="#0065BD"
                )
            )

            # Add vehicle type as text.
            ax.text(
                110 * counter, 80, vehicle_type.name.replace("_", "\n"), ha="center"
            )

            patch_list = icons.get_obstacle_icon_patch(
                obstacle_type=vehicle_type,
                pos_x=110 * counter,
                pos_y=0,
                orientation=0,
                vehicle_length=100,
                vehicle_width=100,
            )
            for patch in patch_list:
                ax.add_patch(patch)
        ax.set_xlim(-60, (len(vehicle_types) - 1) * 110 + 10)
        ax.axis("equal")
        plt.show()

    def test_unsupported_obstacle_type(self):
        """Test if a unsupported obstacle type raises the expected exception."""
        unsupported_obstacle = None
        for obst_type in ObstacleType:
            if obst_type not in icons.supported_icons():
                unsupported_obstacle = obst_type
                break

        if unsupported_obstacle is None:
            assert True
        else:
            self.assertRaises(
                TypeError,
                icons.get_obstacle_icon_patch,
                obstacle_type=unsupported_obstacle,
                pos_x=0,
                pos_y=0,
                orientation=0,
                vehicle_length=100,
                vehicle_width=100,
            )


if __name__ == "__main__":
    unittest.main()
