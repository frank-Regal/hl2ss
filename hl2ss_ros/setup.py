from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['hl2ss_ros'],
    package_dir={'': 'scripts'},
    package_data={'': ['../../viewer/*.py']}
)

setup(**d)