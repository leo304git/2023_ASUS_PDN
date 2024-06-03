add_executable(pd main.cpp)
target_include_directories(pd PRIVATE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(pd PRIVATE base global detailed)

# Add Gurobi
find_package(GUROBI REQUIRED)
target_link_libraries(pd PUBLIC optimized ${GUROBI_CXX_LIBRARY} debug ${GUROBI_CXX_DEBUG_LIBRARY})
target_link_libraries(pd PUBLIC ${GUROBI_LIBRARY})
target_include_directories(pd PRIVATE "${GUROBI_INCLUDE_DIRS}")

# Add Cplex
# find_package(CPLEX REQUIRED)
# target_link_libraries(pd PUBLIC ${CPLEX_LIBRARIES})
# target_link_libraries(pd PUBLIC dl)
# target_include_directories(pd PUBLIC "${CPLEX_INCLUDE_DIRS}")

# Add Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(pd PUBLIC Eigen3::Eigen)

# Add clipp library
#target_link_libraries(pd PRIVATE clipp)
#target_include_directories(pd PRIVATE "${clipp_SOURCE_DIR}"