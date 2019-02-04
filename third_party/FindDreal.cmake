# This sets the following variables:
# DREAL_FOUND - True if dReal was found.
# DREAL_VERSION - dReal Version
# DREAL_INCLUDE_DIRS - Directories containing the dReal include files.
# DREAL_LIBRARY_DIRS - Directories containing the dReal library files.
# DREAL_LIBRARIES - dReal library names.
# DREAL_DEFINITIONS - Compiler flags for dReal.

if(APPLE)
  set(ENV{PKG_CONFIG_PATH} "/usr/local/opt/ibex@2.6.5/share/pkgconfig:$ENV{PKG_CONFIG_PATH}")
  endif(APPLE)
  if(UNIX)
    set(ENV{PKG_CONFIG_PATH} "/opt/dreal/4.18.02.3/lib/pkgconfig:/opt/libibex/2.6.5/share/pkgconfig:$ENV{PKG_CONFIG_PATH}")
    endif(UNIX)

    find_package(PkgConfig)
    pkg_check_modules(DREAL dreal)

    set(DREAL_DEFINITIONS ${DREAL_CFLAGS_OTHER})

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(DREAL
    FOUND_VAR DREAL_FOUND
    REQUIRED_VARS DREAL_INCLUDE_DIRS DREAL_LIBRARIES DREAL_DEFINITIONS
    VERSION_VAR DREAL_VERSION)