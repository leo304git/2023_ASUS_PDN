# Collect file information
file(GLOB source_SRC "*.cpp")
file(GLOB source_HEADER "*.h")

add_library(source ${source_SRC} ${source_HEADER})
target_include_directories(source PUBLIC ${PROJECT_SOURCE_DIR}/src)

# # Add Gurobi
find_package(GUROBI REQUIRED)
#target_link_libraries(network PUBLIC optimized ${GUROBI_CXX_LIBRARY} debug ${GUROBI_CXX_DEBUG_LIBRARY})
#target_link_libraries(network PUBLIC ${GUROBI_LIBRARY})
#target_include_directories(network PRIVATE "${GUROBI_INCLUDE_DIRS}")
find_package(CPLEX REQUIRED)
find_package(Boost 1.85 REQUIRED)

add_subdirectory(base)
add_subdirectory(global)
add_subdirectory(detailed)
