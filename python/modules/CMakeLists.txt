message("Creating python modules")
# Create python module
set(MODULE_NAME "py_${PROJECT_NAME}")
message("\t${MODULE_NAME}")
pybind11_add_module(${MODULE_NAME} MODULE grstapse.cpp)
target_compile_features(${MODULE_NAME}
        PUBLIC
        cxx_std_20)
target_link_libraries(${MODULE_NAME}
        PRIVATE
        Threads::Threads
        _${PROJECT_NAME})
target_compile_options(${MODULE_NAME}
        PRIVATE
        ${_opts})