# - Try to find libtimssdr
# Once done this will define
#  LIBTIMSSDR_FOUND - System has libtimssdr
#  LIBTIMSSDR_INCLUDE_DIRS - The libtimssdr include directories
#  LIBTIMSSDR_LIBRARIES - The libraries needed to use libtimssdr

find_package(PkgConfig)
pkg_check_modules(PC_LIBTIMSSDR QUIET libtimssdr)

find_path(LIBTIMSSDR_INCLUDE_DIR
    NAMES timssdr.h
    HINTS
        $ENV{LIBTIMSSDR_DIR}/include
        ${PC_LIBTIMSSDR_INCLUDEDIR}
        ${PC_LIBTIMSSDR_INCLUDE_DIRS}
    PATH_SUFFIXES libtimssdr
)

find_library(LIBTIMSSDR_LIBRARY
    NAMES timssdr
    HINTS
        $ENV{LIBTIMSSDR_DIR}/lib
        ${PC_LIBTIMSSDR_LIBDIR}
        ${PC_LIBTIMSSDR_LIBRARY_DIRS}
)

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LIBTIMSSDR DEFAULT_MSG LIBTIMSSDR_LIBRARY LIBTIMSSDR_INCLUDE_DIR)

mark_as_advanced(LIBTIMSSDR_INCLUDE_DIR LIBTIMSSDR_LIBRARY)

set(LIBTIMSSDR_INCLUDE_DIRS ${LIBTIMSSDR_INCLUDE_DIR})
set(LIBTIMSSDR_LIBRARIES ${LIBTIMSSDR_LIBRARY})
