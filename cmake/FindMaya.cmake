# Set a default Maya version if not specified
if(NOT DEFINED MAYA_VERSION)
    set(MAYA_VERSION 2020 CACHE STRING "Maya version")
endif()

# OS Specific environment setup
set(MAYA_COMPILE_DEFINITIONS "REQUIRE_IOSTREAM;_BOOL")
set(MAYA_INSTALL_BASE_SUFFIX "")
set(MAYA_TARGET_TYPE LIBRARY)
if(WIN32)
    # Windows
    set(MAYA_INSTALL_BASE_DEFAULT "C:/Program Files/Autodesk")
    set(MAYA_COMPILE_DEFINITIONS "${MAYA_COMPILE_DEFINITIONS};NT_PLUGIN")
    set(MAYA_PLUGIN_EXTENSION ".mll")
    set(MAYA_TARGET_TYPE RUNTIME)
elseif(APPLE)
    # Apple
    set(MAYA_INSTALL_BASE_DEFAULT /Applications/Autodesk)
    set(MAYA_COMPILE_DEFINITIONS "${MAYA_COMPILE_DEFINITIONS};OSMac_")
    set(MAYA_PLUGIN_EXTENSION ".bundle")
else()
    # Linux
    set(MAYA_COMPILE_DEFINITIONS "${MAYA_COMPILE_DEFINITIONS};LINUX")
    set(MAYA_INSTALL_BASE_DEFAULT /usr/autodesk)
    if(MAYA_VERSION LESS 2016)
        # Pre Maya 2016 on Linux
        set(MAYA_INSTALL_BASE_SUFFIX -x64)
    endif()
    set(MAYA_PLUGIN_EXTENSION ".so")
endif()

set(MAYA_INSTALL_BASE_PATH ${MAYA_INSTALL_BASE_DEFAULT} CACHE STRING
    "Root path containing your maya installations, e.g. /usr/autodesk or /Applications/Autodesk/")

set(MAYA_LOCATION ${MAYA_INSTALL_BASE_PATH}/maya${MAYA_VERSION}${MAYA_INSTALL_BASE_SUFFIX})

# Maya include directory
find_path(MAYA_INCLUDE_DIR maya/MFn.h
    PATHS
        ${MAYA_LOCATION}
        $ENV{MAYA_LOCATION}
    PATH_SUFFIXES
        "include/"
        "devkit/include/"
)

find_library(MAYA_LIBRARY
    NAMES 
        OpenMaya
    PATHS
        ${MAYA_LOCATION}
        $ENV{MAYA_LOCATION}
    PATH_SUFFIXES
        "lib/"
        "Maya.app/Contents/MacOS/"
    NO_DEFAULT_PATH
)
set(MAYA_LIBRARIES "${MAYA_LIBRARY}")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Maya
    REQUIRED_VARS MAYA_INCLUDE_DIR MAYA_LIBRARY)
mark_as_advanced(MAYA_INCLUDE_DIR MAYA_LIBRARY)

if (NOT TARGET Maya::Maya)
    add_library(Maya::Maya UNKNOWN IMPORTED)
    set_target_properties(Maya::Maya PROPERTIES
        INTERFACE_COMPILE_DEFINITIONS "${MAYA_COMPILE_DEFINITIONS}"
        INTERFACE_INCLUDE_DIRECTORIES "${MAYA_INCLUDE_DIR}"
        IMPORTED_LOCATION "${MAYA_LIBRARY}")

    if (APPLE AND ${CMAKE_CXX_COMPILER_ID} MATCHES "Clang" AND MAYA_VERSION LESS 2017)
        # Clang and Maya 2016 and older needs to use libstdc++
        set_target_properties(Maya::Maya PROPERTIES
            INTERFACE_COMPILE_OPTIONS "-std=c++0x;-stdlib=libstdc++")
    endif ()
endif()

# Add the other Maya libraries into the main Maya::Maya library
set(_MAYA_LIBRARIES OpenMayaAnim OpenMayaFX OpenMayaRender OpenMayaUI Foundation clew)
foreach(MAYA_LIB ${_MAYA_LIBRARIES})
    find_library(MAYA_${MAYA_LIB}_LIBRARY
        NAMES 
            ${MAYA_LIB}
        PATHS
            ${MAYA_LOCATION}
            $ENV{MAYA_LOCATION}
        PATH_SUFFIXES
            "lib/"
            "Maya.app/Contents/MacOS/"
        NO_DEFAULT_PATH)
    mark_as_advanced(MAYA_${MAYA_LIB}_LIBRARY)
    if (MAYA_${MAYA_LIB}_LIBRARY)
        add_library(Maya::${MAYA_LIB} UNKNOWN IMPORTED)
        set_target_properties(Maya::${MAYA_LIB} PROPERTIES
            IMPORTED_LOCATION "${MAYA_${MAYA_LIB}_LIBRARY}")
        set_property(TARGET Maya::Maya APPEND PROPERTY
            INTERFACE_LINK_LIBRARIES Maya::${MAYA_LIB})
        set(MAYA_LIBRARIES ${MAYA_LIBRARIES} "${MAYA_${MAYA_LIB}_LIBRARY}")
    endif()
endforeach()

function(MAYA_PLUGIN _target)
    if (WIN32)
        set_target_properties(${_target} PROPERTIES
            LINK_FLAGS "/export:initializePlugin /export:uninitializePlugin")
    endif()
    set_target_properties(${_target} PROPERTIES
        PREFIX ""
        SUFFIX ${MAYA_PLUGIN_EXTENSION})
endfunction()