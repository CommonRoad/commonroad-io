import unittest

import numpy as np

from commonroad.geometry.transform import translation_rotation_matrix


class TestTranslationRotationMatrix(unittest.TestCase):
    def test_rotate_90deg(self):
        translation = np.array([0.0, 0.0])
        rotation_angle = np.pi / 2

        homogenous_initial_vector = np.array([1, 1, 1])
        expected_transposed_vector = np.array([-1, 1, 1])

        transposed_vector = translation_rotation_matrix(translation, rotation_angle).dot(homogenous_initial_vector)

        for i, elt in enumerate(transposed_vector):
            self.assertAlmostEqual(elt, expected_transposed_vector[i])

    def test_translate(self):
        translation = np.array([5.52, -2.2])
        rotation_angle = 0

        homogenous_initial_vector = np.array([1, 1, 1])
        expected_transposed_vector = np.array([6.52, -1.2, 1])

        transposed_vector = translation_rotation_matrix(translation, rotation_angle).dot(homogenous_initial_vector)

        for i, elt in enumerate(transposed_vector):
            self.assertAlmostEqual(elt, expected_transposed_vector[i])


if __name__ == "__main__":
    unittest.main()
