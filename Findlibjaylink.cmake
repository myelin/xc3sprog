# Copyright 2017 Google Inc.
# Phillip Pearson <philpearson@google.com>
# Copyright 2009  SoftPLC Corporation  http://softplc.com
# Dick Hollenbeck <d...@softplc.com>
# License: GPL v2
#
# - Try to find libjaylink
#
# Once done this will define
#
# LIBJAYLINK_FOUND - system has libjaylink
# LIBJAYLINK_INCLUDE_DIR - the libjaylink include directory
# LIBJAYLINK_LIBRARIES - Link these to use libjaylink


if (NOT LIBJAYLINK_FOUND)

    if(NOT WIN32)
        include(FindPkgConfig)
        pkg_check_modules(LIBJAYLINK_PKG libjaylink)
    endif(NOT WIN32)

    find_path(LIBJAYLINK_INCLUDE_DIR
        NAMES
            libjaylink.h
        HINTS
            ${LIBJAYLINK_PKG_INCLUDE_DIRS}
        PATHS
            /usr/include/libjaylink
            /usr/local/include/libjaylink
    )

    if(USE_STATIC_FTDI)
        set(_save ${CMAKE_FIND_LIBRARY_SUFFIXES})
        set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_STATIC_LIBRARY_SUFFIX})
    endif(USE_STATIC_FTDI)

    find_library(LIBJAYLINK_LIBRARIES
        NAMES
            jaylink
        HINTS
            ${LIBJAYLINK_PKG_LIBRARY_DIRS}
        PATHS
            /usr/lib
            /usr/local/lib
    )

    if(USE_STATIC_FTDI)
        set(CMAKE_FIND_LIBRARY_SUFFIXES ${_save} )
    endif(USE_STATIC_FTDI)
    include(FindPackageHandleStandardArgs)

    # handle the QUIETLY AND REQUIRED arguments AND set LIBJAYLINK_FOUND to TRUE if
    # all listed variables are TRUE
    find_package_handle_standard_args(LIBJAYLINK DEFAULT_MSG LIBJAYLINK_LIBRARIES LIBJAYLINK_INCLUDE_DIR)

    if(USE_STATIC_FTDI)
        add_library(libjaylink STATIC IMPORTED)
    else(USE_STATIC_FTDI)
        add_library(libjaylink SHARED IMPORTED)
    endif(USE_STATIC_FTDI)

    set_target_properties(libjaylink PROPERTIES IMPORTED_LOCATION ${LIBJAYLINK_LIBRARIES})
    set(${LIBJAYLINK_LIBRARIES} libjaylink)

    #mark_as_advanced(LIBJAYLINK_INCLUDE_DIR LIBJAYLINK_LIBRARIES)

endif(NOT LIBJAYLINK_FOUND)
