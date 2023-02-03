from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
setup_args = generate_distutils_setup(
    packages=['ros_param_example'],
    package_dir={'': 'scripts'}
)
setup(**setup_args)
