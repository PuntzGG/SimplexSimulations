cmake_minimum_required(VERSION 3.20)
project(SimplexScientificCoreTests LANGUAGES CXX)

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../replacements/LogitDynamics.cpp")
    set(SIMPLEX_ROOT "${CMAKE_CURRENT_LIST_DIR}/../replacements")
else()
    set(SIMPLEX_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")
endif()

add_executable(SimplexScientificCoreTests
    ScientificCoreTests.cpp
    "${SIMPLEX_ROOT}/LogitDynamics.cpp"
    "${SIMPLEX_ROOT}/LogitEquilibriumSweep.cpp"
    "${SIMPLEX_ROOT}/SimulationSession.cpp"
    "${SIMPLEX_ROOT}/SimplexEquilibriumFinder.cpp"
    "${SIMPLEX_ROOT}/SimplexTrajectoryIntegrator.cpp"
)

target_include_directories(SimplexScientificCoreTests PRIVATE
    "${SIMPLEX_ROOT}"
)

target_compile_features(SimplexScientificCoreTests PRIVATE cxx_std_20)

if(MSVC)
    target_compile_options(SimplexScientificCoreTests PRIVATE /W4 /WX /permissive-)
else()
    target_compile_options(SimplexScientificCoreTests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror
    )
endif()

enable_testing()
add_test(NAME ScientificCore COMMAND SimplexScientificCoreTests)
