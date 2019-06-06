Development
============

The repository is structured in master, develop, and feature branches, which are organized as described here: https://nvie.com/posts/a-successful-git-branching-model/.
For coding standards, refer to the guide in commonroad/style_guide.rst

 
Requirements
============

Following packages are necessary to build the documentation:

* graphviz>=0.3
* sphinx>=1.3.6
* sphinx-autodoc-typehints>=1.3.0
* sphinx_rtd_theme>=0.4.1
* sphinx-gallery>=0.2.0
* ipython>=6.5.0

Following additional packages are necessary to run the unit tests:

* lxml>=4.2.5
* pytest>=3.8.0

Following additional packages are necessary to run the tutorials:

* cvxpy==0.4.9
* jupyter>=1.0.0

Installation with Anaconda
--------------------------

#. Install Anaconda_.

#. Open your console in the root folder of the CommonRoad repository.

#. Create a new Anaconda environment for Python 3.6. Run in your Terminal window
	
	.. code-block:: console

		   $ conda create -n commonroad-py36 python=3.6

   Note that you have to replace *commonroad-py36* with the name of the environment you want to create. 

#. Activate your environment with 

	.. code-block:: console

		   $ source activate commonroad-py36

#. Change directory to the CommonRoad Python-Tools:

	.. code-block:: console

		   $ cd tools/Python/

#. Install all requirements:

    .. code-block:: console
        
		   $ pip install -r requirements.txt

    and add tools/Python to your Python interpreter. 

    **OR**

    Run setup.py to install the package

	.. code-block:: console

		   $ python setup.py install

Unit Tests
----------

If you want to execute the available unit tests, following commands are necessary:

#. Change directory to the CommonRoad Python-Tools:

	.. code-block:: console

		   $ cd tools/Python/

#. Activate your environment with 

	.. code-block:: console

		   $ source activate commonroad-py36

#. Install the required packages with

	.. code-block:: console

		   $ pip install -e .[tests]

Documentation
-------------

The user manual can be generated with the following commands:

#. Change directory to the CommonRoad Python-Tools:

	.. code-block:: console

		   $ cd tools/Python/

#. Activate your environment with 

	.. code-block:: console

		   $ source activate commonroad-py36

#. Install the required packages with

	.. code-block:: console

		   $ pip install -e .[doc]

#. Build the documentation with

	.. code-block:: console

		   $ python setup.py build_sphinx

Tutorials
---------

If you want to execute the available tutorials, following commands are necessary:

#. Change directory to the CommonRoad Python-Tools:

	.. code-block:: console

		   $ cd tools/Python/

#. Activate your environment with 

	.. code-block:: console

		   $ source activate commonroad-py36

#. Install the required packages with

	.. code-block:: console

		   $ pip install -e .[tutorials]


Gitlab CI/CD
==============

To set up Gitlab CI/CD, follow these instructions:


#. Activate Pipelines in Settings->General->Visibility, project features, permissions

#. Under Settings->CI/CD you can make adjustments.

#. Set up a new gitlab runner (necessary Token and Address are at Settings->CI/CD->Runners)

 * Generally the document describing the install process is at https://docs.gitlab.com/runner/install/
 
 * In the case of CommonRoad, gitlab-runner is running inside a docker container: https://docs.gitlab.com/runner/install/docker.html
 
 * The following two commands (quite similar to the instructions) were used to set up several docker containers with gitlab-runners:
 
    .. code-block:: console

        $ docker run -d --name <project-name>-gitlab-runner --restart always -v /var/run/docker.sock:/var/run/docker.sock -v <project-name>-data:/etc/gitlab-runner gitlab/gitlab-runner:latest

        $ docker run --rm -t -i -v <project-name>-data:/etc/gitlab-runner gitlab/gitlab-runner register
 
  The first command starts the runner with name <project-name>-gitlab-runner and mounts a new volume which contains the gitlab-runner configuration.
  The second command runs a temporary container for the registration process, which saves the result in the shared docker volume <project-name>-data.
