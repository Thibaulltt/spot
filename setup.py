# SPOT/setup.py : Set up the right dependencies for the python module to be generated.
import sys, os

# Available at setup time due to pyproject.toml
from pybind11 import get_cmake_dir
from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

__version__ = "0.0.5"

# The main interface is through Pybind11Extension.
# * You can add cxx_std=11/14/17, and then build_ext can be removed.
# * You can set include_pybind11=false to add the include directory yourself,
#   say from a submodule.
#
# Note:
#   Sort input source files if you glob sources to ensure bit-for-bit
#   reproducible builds (https://github.com/pybind/python_example/pull/53)

os.environ['CC']='clang'
os.environ['CXX']='clang++'

ext_modules = [
	Pybind11Extension("_spot",
		["./src/spot_wrappers_python_interface.cpp",
			"./src/spot_wrappers.hpp",
			"./src/spot_wrappers.cpp",
		],
		include_dirs=["./external"],
		# Example: passing in the version to the compiled code
		define_macros=[('VERSION_INFO', __version__)],
		libraries=["openmp", "glm", "fmt"],
		extra_compile_args=[
			"-Wno-unknown-pragmas",
		],
		include_pybind11=True,
		cxx_std=14,
		language='c++'
	),
]

kwargs = dict(
	name="spot",
	version=__version__,
	author='Bonneel N., Coeurjolly D., de Villèle T.',
	author_email='thibault.de-villele@umontpellier.fr',
	description='A set of Python wrappers around the SPOT method (Bonneel, Coeurjolly @ SIGGRAPH 2019)',
	ext_modules=ext_modules,
	cmdclass=dict(build_ext=build_ext),
	zip_safe=False,
	python_requires=">=3.7",
	packages=["spot"]
)

setup(**kwargs)

