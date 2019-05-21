from setuptools import setup, find_packages
from os import path

# read the README file
from os import path
this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, 'README.rst'), encoding='utf-8') as f:
    readme = f.read()


setup(
    name='commonroad-io',
    version='2019.1',
    description='Python tool to read, write, and visualize CommonRoad scenarios and solutions.',
    keywords = 'autonomous automated vehicles driving motion planning',
    url='https://commonroad.in.tum.de/',
    author='Technical University of Munich',  
    author_email='commonroad-i06@in.tum.de',
    license="GNU General Public License v3.0",
    packages=find_packages(exclude=['doc', 'tests', 'checker','tutorials']), 
	install_requires=[
		'numpy>=1.13',
		'Shapely>=1.6.4',
		'matplotlib>=3.0.0',
		'lxml>=4.2.5',
		'networkx>=2.2'
	],
	extras_require={
		'doc':	['sphinx>=1.3.6',
				 'graphviz>=0.3',
				 'sphinx-autodoc-typehints>=1.3.0',
                 'sphinx_rtd_theme>=0.4.1',
                 'sphinx-gallery>=0.2.0',
                 'ipython>=6.5.0'],
		'tests': ['lxml>=4.2.5',
				  'pytest>=3.8.0',],
        'tutorials': ['cvxpy==0.4.9',
                      'jupyter>=1.0.0'],
	},
    long_description_content_type='text/x-rst',
    long_description=readme,    
    classifiers=[
	"Programming Language :: Python :: 3.6",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
	"Operating System :: POSIX :: Linux",
        "Operating System :: MacOS",	
    ],
    data_files=[('.',['LICENSE.txt'])],
    include_package_data=True,
)
