message("Building python interfaces (cpp -> python AND python -> cpp)")
add_subdirectory(modules)

add_custom_target(copy_py${PROJECT_NAME}_for_tests
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/modules/py_${PROJECT_NAME} ${CMAKE_CURRENT_BINARY_DIR}/tests/py_${PROJECT_NAME}
        DEPENDS py_${PROJECT_NAME})

add_custom_target(run_python_tests
        COMMENT "Running 'python -m unittest discover -s ${CMAKE_CURRENT_BINARY_DIR}/tests'"
        COMMAND python3 -m unittest discover -s ${CMAKE_CURRENT_BINARY_DIR}/tests
        DEPENDS py_${PROJECT_NAME} copy_py${PROJECT_NAME}_for_tests)