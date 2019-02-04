# - Try to find Z3
# Once done this will define
#  Z3_FOUND - System has Z3
#  Z3_INCLUDE_DIRS - The Z3 include directories
#  Z3_LIBRARIES - The libraries needed to use Z3

if (Z3_HOME)
  find_path(Z3_INCLUDE_DIR z3.h PATHS "${Z3_HOME}/include")
else()
  find_path(Z3_INCLUDE_DIR z3.h)
endif()

if (Z3_HOME)
  find_library(Z3_LIBRARY z3 PATHS "${Z3_HOME}/lib")
else()
  find_library(Z3_LIBRARY z3)
endif()


set(Z3_LIBRARIES ${Z3_LIBRARY})
set(Z3_INCLUDE_DIRS ${Z3_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Z3 DEFAULT_MSG Z3_LIBRARY Z3_INCLUDE_DIR)

mark_as_advanced(Z3_INCLUDE_DIR Z3_LIBRARY)