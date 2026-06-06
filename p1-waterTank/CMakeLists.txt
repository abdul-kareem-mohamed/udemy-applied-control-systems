cmake_minimum_required(VERSION 3.8)

project(WaterTankController LANGUAGES CXX)

# Export compilation commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# ---- Python ----
find_package(Python3 REQUIRED COMPONENTS Interpreter)

execute_process(COMMAND ${Python3_EXECUTABLE} -c "import pytest" RESULT_VARIABLE _pytest_found OUTPUT_QUIET ERROR_QUIET)
if(_pytest_found EQUAL 0)
  set(HAVE_PYTEST TRUE)
else()
  set(HAVE_PYTEST FALSE)
endif()

# ---- targets ----
add_library(pid_controller STATIC 
  src/pid_controller.cpp
)

target_include_directories(pid_controller 
  PUBLIC 
    include
)

add_executable(water_tank_controller 
  src/water_tank_controller.cpp
)

target_compile_features(water_tank_controller PRIVATE cxx_std_17)

target_link_libraries(water_tank_controller PRIVATE pid_controller)

# Use `cmake --build . --target run_python_scripts` to run scripts manually.
add_custom_target(run_python_scripts
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/src/p1-waterTank/controller.py
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/src/p1-waterTank/tank_sim.py
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/src/p1-waterTank/visualizer.py
  
  COMMENT "Executing Python scripts sequentially..."
  VERBATIM
  )

  # for pytest
  enable_testing()

  if(HAVE_PYTEST)
    add_test(NAME pytest_tests
      COMMAND ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/tests
    )

  else()
    message(WARNING "pytest not found in Python; 'ctest' will report no tests. Install pytest to enable test running.")
  endif()

add_custom_target(run_pytests
  COMMAND ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/tests
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Run pytest tests"
  VERBATIM
)