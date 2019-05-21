Python StyleGuide for CommonRoad 
==================================

This guideline has been developed in order to enable developers to produce code which is readable, maintainable, easy to understand, well documented, and efficient. Anyone who wants to participate in the development of the CommonRoad Python framework needs to follow the defined rules. If someone commits code which is not in accordance to the guide, he or she will receive a warning and a deadline until the code must have been refactored. If this deadline has been passed without proper fixes, the developer must bring a cake, a round of beer, savory food or other stuff to apologize for any inconvenience.

The style guide is structured into different areas, such as coding, testing, and exception handling. If important points are missing in any section or a section should be added to the guide, feel free to create an issue to discuss the points with the other developers. 

Formatting
----------
1. We use the PEP8 standard (see https://www.python.org/dev/peps/pep-0008/)
2. Classes / methods / variables are named in a meaningful way
3. We make use of a requirements.txt file to automatically download required Python packages via pip

Commenting
----------
1. Every file contains information about the authors:

.. code-block:: python

	__author__ = "Christian Pek"
	__copyright__ = "TUM Cyber-Physical System Group"
	__credits__ = ["BMW Group CAR@TUM"]
	__version__ = "0.9"
	__maintainer__ = "Christian Pek"
	__email__ = "Christian.Pek@tum.de"
	__status__ = "Released"

2. The functionality of every class is documented:


.. code-block:: python

	class QPLatState(object):
	"""
	Class representing a state <d,theta,kappa,theta_ref> within the QPLatPlanner
	"""

3. Every method gets a description with provided input and computed output:

.. code-block:: python

	def compute_curvature_from_polyline(polyline: npy.ndarray) -> npy.ndarray:
	"""
	Computes the curvature of a given polyline
	:param polyline: The polyline for the curvature computation
	:return: The curvature of the polyline
	"""

4. Produced code is commented such that new developers can easily understand what you have programmed




Coding
------

1. We make use of enhanced Python functionalities, such as properties and existing modules
2. Code is designed to be computationally efficient (use run-time measurements)
3. Produce modular code so that other developers can use parts of your code for their own modules
4. Think object-oriented and design classes and relations accordingly
5. We make use of typing to provide typing support in IDEs
6. We do not reinvent the wheel and check if the wanted functionality already exists within our software or a Python package



Exception Handling
------------------

1. We use assertions to check the correctness of provided inputs
2. If something goes terribly wrong, we throw an exception with an appropriate error message
3. The software performs data integrity operations before an exception will be thrown
4. We use warnings to inform users if results are valid but less meaningful
 

Testing
-------

1. We make use of the unittest Python package to provide enhanced testing (see https://docs.python.org/3/library/unittest.html)
2. Every functionality gets a test
3. Basic tests are: 
	* what happens if correct/incorrect input is provided? 
	* has the expected output been computed?
	* what happens if different values for valid inputs are provided (e.g. small and large numbers)?
4. Complex tests are:
	* Is the implementation of the class according to its specification?
	* Has every functionality of the class been tested?
	* Does the class produce expected outputs?
5. Code is not allowed to be committed if tests fail 
6. New tests (even in other packages) are added if new functionalities require new test cases
7. If the code of another developer contains bugs, we add a new issue with a detailed problem description and example code to reproduce the bug


