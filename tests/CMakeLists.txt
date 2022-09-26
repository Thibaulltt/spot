# This performs some tests on the classes designed in the project root.

ENABLE_TESTING()

SET(SPOT_BASE_TEST_DIR "${CMAKE_SOURCE_DIR}/")
CONFIGURE_FILE(path_setup.hpp.in path_setup.hpp @ONLY)

ADD_SUBDIRECTORY(fist_method)
ADD_SUBDIRECTORY(models)
ADD_SUBDIRECTORY(python_interface)
