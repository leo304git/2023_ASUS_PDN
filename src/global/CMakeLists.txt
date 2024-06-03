# Collect file information
file(GLOB global_SRC "*.cpp")
file(GLOB global_HEADER "*.h")

add_library(global ${global_SRC} ${global_HEADER})
target_include_directories(global PUBLIC ${PROJECT_SOURCE_DIR}/src/global)
target_link_libraries(global PRIVATE base)

# Add Gurobi
find_package(GUROBI REQUIRED)
target_link_libraries(global PUBLIC optimized ${GUROBI_CXX_LIBRARY} debug ${GUROBI_CXX_DEBUG_LIBRARY})
target_link_libraries(global PUBLIC ${GUROBI_LIBRARY})
target_include_directories(global PRIVATE "${GUROBI_INCLUDE_DIRS}")

# Add Cplex
# find_package(CPLEX REQUIRED)
# target_link_libraries(global PUBLIC ${CPLEX_LIBRARIES})
# target_link_libraries(global PUBLIC dl)
# target_include_directories(global PUBLIC "${CPLEX_INCLUDE_DIRS}")