cmake_minimum_required(VERSION 3.16)
message("Building executables")

function(subdirectory_list RESULT DIRECTORY)
    file(GLOB CHILDREN RELATIVE ${DIRECTORY} ${DIRECTORY}/*)
    set(DIRECTORY_LIST "")
    foreach (CHILD ${CHILDREN})
        if (IS_DIRECTORY ${DIRECTORY}/${CHILD})
            list(APPEND DIRECTORY_LIST ${CHILD})
        endif ()
    endforeach (CHILD ${CHILDREN})
    set(${RESULT} ${DIRECTORY_LIST} PARENT_SCOPE)
endfunction()
subdirectory_list(SUBDIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR})

foreach (SUBDIRECTORY ${SUBDIRECTORIES})
    if (NOT "${SUBDIRECTORY}" STREQUAL "data")
        message("\tBuilding ${SUBDIRECTORY}")

        # Collect any header and source files
        set(EXECUTABLE_HEADERS "")
        set(EXECUTABLE_SOURCES "")
        file(GLOB EXECUTABLE_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIRECTORY}/*.hpp)
        file(GLOB EXECUTABLE_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIRECTORY}/*.cpp)

        add_executable(${SUBDIRECTORY})
        target_sources(${SUBDIRECTORY}
                PRIVATE
                ${EXECUTABLE_HEADERS}
                ${EXECUTABLE_SOURCES})
        target_compile_features(${SUBDIRECTORY}
                PUBLIC
                cxx_std_20)
        target_link_libraries(${SUBDIRECTORY}
                PRIVATE
                Threads::Threads
                _${PROJECT_NAME})
        target_compile_options(${SUBDIRECTORY}
                PRIVATE
                ${_opts})
    endif (NOT "${SUBDIRECTORY}" STREQUAL "data")
endforeach ()