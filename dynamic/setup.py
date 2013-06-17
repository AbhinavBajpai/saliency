
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
      packages=['dynamic'],
      package_dir = {'':'src'},
      install_requires=['ros_comm'],
      scripts = ['scripts/rat']
)

setup(**setup_args)
