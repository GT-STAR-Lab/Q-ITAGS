find_path(GUROBI_INCLUDE_DIRECTORY
        NAMES
        gurobi_c.h
        gurobi_c++.h
        HINTS
        ${GUROBI_DIR}
        $ENV{GUROBI_HOME}
        /opt/gurobi/linux64
        /opt/gurobi911/linux64
        /opt/gurobi912/linux64
        /opt/gurobi950/linux64
        PATH_SUFFIXES
        include)

find_library(GUROBI_LIBRARY
        NAMES
        gurobi
        gurobi91
        gurobi95
        HINTS
        ${GUROBI_DIR}
        $ENV{GUROBI_HOME}
        /opt/gurobi/linux64
        /opt/gurobi911/linux64
        /opt/gurobi912/linux64
        /opt/gurobi950/linux64
        PATH_SUFFIXES
        lib)

find_library(GUROBI_CXX_LIBRARY
        NAMES
        gurobi_c++
        HINTS
        ${GUROBI_DIR}
        $ENV{GUROBI_HOME}
        /opt/gurobi/linux64
        /opt/gurobi911/linux64
        /opt/gurobi912/linux64
        /opt/gurobi950/linux64
        PATH_SUFFIXES
        lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(gurobi REQUIRE_VARS GUROBI_CXX_LIBRARY GUROBI_LIBRARY GUROBI_INCLUDE_DIRECTORY)

if (GUROBI_CXX_LIBRARY AND GUROBI_LIBRARY AND GUROBI_INCLUDE_DIRECTORY)
    set(gurobi_FOUND TRUE)
endif (GUROBI_CXX_LIBRARY AND GUROBI_LIBRARY AND GUROBI_INCLUDE_DIRECTORY)

if (gurobi_FOUND)
    add_library(gurobi_c UNKNOWN IMPORTED GLOBAL)
    set_target_properties(gurobi_c PROPERTIES
            IMPORTED_CONFIGURATIONS RELEASE
            IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
            IMPORTED_LOCATION_RELEASE ${GUROBI_LIBRARY}
            INTERFACE_INCLUDE_DIRECTORIES ${GUROBI_INCLUDE_DIRECTORY}
            IMPORTED_GLOBAL ON)

    add_library(gurobi UNKNOWN IMPORTED GLOBAL)
    set_target_properties(gurobi PROPERTIES
            IMPORTED_CONFIGURATIONS RELEASE
            IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
            IMPORTED_LOCATION_RELEASE ${GUROBI_CXX_LIBRARY}
            INTERFACE_LINK_LIBRARIES gurobi_c
            INTERFACE_INCLUDE_DIRECTORIES ${GUROBI_INCLUDE_DIRECTORY}
            IMPORTED_GLOBAL ON)
endif (gurobi_FOUND)