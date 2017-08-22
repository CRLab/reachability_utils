## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# requirements = [line.strip() for line in open("requirements.txt")]

d = generate_distutils_setup()

d['name'] = "reachability_utils"
d['description'] = "python package for reachability_utils"
d['packages'] = ['reachability_utils']
d['package_dir'] = {'': 'scripts'}
# d['requirements'] = requirements

setup(**d)