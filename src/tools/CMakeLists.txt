include_directories( libsurvive )

add_executable(config_brute_force config_brute_force.cc)
target_link_libraries(config_brute_force solvers)
