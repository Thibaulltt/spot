# -------------
# SPOT/setup.py : Set up the right dependencies for the python module to be generated.
# -------------

import os
import re
import subprocess
import sys
from pathlib import Path

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext

module_information = dict(
	author='Bonneel, Coeurjolly, de Villèle',
	author_email='thibault.de-villele@umontpellier.fr',
	version='0.0.5',
	description='',
)

# Force the use of the LLVM compiler suite :
os.environ['CC'] = 'clang'
os.environ['CXX'] = 'clang++'

# A CMakeExtension needs a sourcedir instead of a file list.
# The name must be the _single_ output extension from the CMake build.
# If you need multiple extensions, see scikit-build.
class CMakeExtension(Extension):
	def __init__(self, name: str, sourcedir: str = "", *args, **kwargs) -> None:
		super().__init__(name, *args, sources=[], **kwargs)
		self.sourcedir = os.fspath(Path(sourcedir).resolve())


class CMakeBuild(build_ext):
	def build_extension(self, ext: CMakeExtension) -> None:
		# Must be in this form due to bug in .resolve() only fixed in Python 3.10+
		ext_fullpath = Path.cwd() / self.get_ext_fullpath(ext.name)  # type: ignore[no-untyped-call]
		extdir = ext_fullpath.parent.resolve()

		# Using this requires trailing slash for auto-detection & inclusion of
		# auxiliary "native" libs

		debug = int(os.environ.get("DEBUG", 0)) if self.debug is None else self.debug
		cfg = "Debug" if debug else "Release"

		# CMake lets you override the generator - we need to check this.
		# Can be set with Conda-Build, for example.
		cmake_generator = os.environ.get("CMAKE_GENERATOR", "")

		# Set Python_EXECUTABLE instead if you use PYBIND11_FINDPYTHON
		# EXAMPLE_VERSION_INFO shows you how to pass a value into the C++ code
		# from Python.
		cmake_args = [
			f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}{os.sep}",
			f"-DPYTHON_EXECUTABLE={sys.executable}",
			f"-DCMAKE_BUILD_TYPE={cfg}",  # not used on MSVC, but no harm
		]
		build_args = []
		# Adding CMake arguments set as environment variable
		# (needed e.g. to build for ARM OSx on conda-forge)
		if "CMAKE_ARGS" in os.environ:
			cmake_args += [item for item in os.environ["CMAKE_ARGS"].split(" ") if item]

		# In this example, we pass in the version to C++. You might not need to.
		cmake_args += [f"-DVERSION_INFO=\"{module_information['version']}\""]  # type: ignore[attr-defined]

		# Using Ninja-build since it a) is available as a wheel and b)
		# multithreads automatically. MSVC would require all variables be
		# exported for Ninja to pick it up, which is a little tricky to do.
		# Users can override the generator with CMAKE_GENERATOR in CMake
		# 3.15+.
		if not cmake_generator or cmake_generator == "Ninja":
			try:
				import ninja  # noqa: F401

				ninja_executable_path = Path(ninja.BIN_DIR) / "ninja"
				cmake_args += [
					"-GNinja",
					f"-DCMAKE_MAKE_PROGRAM:FILEPATH={ninja_executable_path}",
				]
			except ImportError:
				pass

		if sys.platform.startswith("darwin"):
			# Cross-compile support for macOS - respect ARCHFLAGS if set
			archs = re.findall(r"-arch (\S+)", os.environ.get("ARCHFLAGS", ""))
			if archs:
				cmake_args += ["-DCMAKE_OSX_ARCHITECTURES={}".format(";".join(archs))]

		# Set CMAKE_BUILD_PARALLEL_LEVEL to control the parallel build level
		# across all generators.
		if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
			# self.parallel is a Python 3 only way to set parallel jobs by hand
			# using -j in the build_ext call, not supported by pip or PyPA-build.
			if hasattr(self, "parallel") and self.parallel:
				# CMake 3.12+ only.
				build_args += [f"-j{self.parallel}"]

		if "CMAKE_ENABLE_TESTS" in os.environ:
			cmake_args += ["-DCMAKE_ENABLE_TESTS=On"]

		build_temp = Path(self.build_temp) / ext.name
		if not build_temp.exists():
			build_temp.mkdir(parents=True)

		subprocess.run(
			["cmake", ext.sourcedir] + cmake_args, cwd=build_temp, check=True
		)
		subprocess.run(
			["cmake", "--build", "."] + build_args, cwd=build_temp, check=True
		)


# The information here can also be placed in setup.cfg - better separation of
# logic and declaration, and simpler if you include description/version in a file.
setup(
	**module_information,
	packages=["spot"],
	ext_modules=[CMakeExtension("spot._spot")],
	cmdclass={"build_ext": CMakeBuild},
	zip_safe=False,
	python_requires=">=3.7",
)
