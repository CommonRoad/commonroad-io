# read the README file
from os import path

from setuptools import setup, find_packages

this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, 'README.rst'), encoding='utf-8') as f:
    readme = f.read()

setup(name='commonroad-io', version='2021.4',
      description='Python tool to read, write, and visualize CommonRoad scenarios and solutions for automated '
                  'vehicles.',
      keywords='autonomous automated vehicles driving motion planning', url='https://commonroad.in.tum.de/',
      project_urls={'Documentation': 'https://commonroad.in.tum.de/static/docs/commonroad-io/index.html',
                    'Forum': 'https://commonroad.in.tum.de/forum/c/commonroad-io',
                    'Source': 'https://gitlab.lrz.de/tum-cps/commonroad_io', },
      author='Cyber-Physical Systems Group, Technical University of Munich', author_email='commonroad@lists.lrz.de',
      license="BSD", packages=find_packages(exclude=['doc', 'tests', 'tutorials']),
      install_requires=['numpy>=1.13', 'scipy>=1.5.2', 'shapely>=1.6.4', 'matplotlib>=3.0.0', 'lxml>=4.2.2',
                        'networkx>=2.2', 'Pillow>=7.0.0', 'iso3166>=1.0.1', 'commonroad-vehicle-models>=2.0.0',
                        'rtree>=0.8.3',
                        ],
      extras_require={
          'doc': ['sphinx>=1.3.6',
                  'graphviz>=0.3',
                  'sphinx-autodoc-typehints>=1.3.0',
                  'sphinx_rtd_theme>=0.4.1',
                  'sphinx-gallery>=0.2.0',
                  'ipython>=6.5.0'],
          'tests': ['lxml>=4.2.5',
                    'pytest>=3.8.0', ],
          'tutorials': ['cvxpy==0.4.9',
                        'jupyter>=1.0.0'],
      },
      long_description_content_type='text/x-rst',
      long_description=readme,
      classifiers=["Programming Language :: Python :: 3.6", "License :: OSI Approved :: BSD License",
                   "Operating System :: POSIX :: Linux", "Operating System :: MacOS", ],
      data_files=[('.', ['LICENSE.txt'])],
      include_package_data=True,
      )
