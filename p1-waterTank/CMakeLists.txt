cmake_minimum_required(VERSION 3.8)

project(WaterTankController LANGUAGES CXX)

# Export compilation commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# ---- Options ----
option(USE_CPP_CONTROLLER "Use C++ executable for the controller instead of Python" ON)

# ---- CPP ----
include(FetchContent)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

if (NOT spdlog_FOUND)
  FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.14.1
  )
  FetchContent_MakeAvailable(spdlog)
endif()

if (NOT fmt_FOUND)
  FetchContent_Declare(
    fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt.git
    GIT_TAG 11.0.1
  )
  FetchContent_MakeAvailable(fmt)
endif()

if (NOT nlohmann_json)
  FetchContent_Declare(nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.12.0
  )
  FetchContent_MakeAvailable(nlohmann_json)
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

target_link_libraries(water_tank_controller PRIVATE 
  pid_controller 
  spdlog::spdlog 
  fmt::fmt-header-only
  nlohmann_json::nlohmann_json
)

# ctest
# run runing `ctest` inside the build dir


# ---- PYTHON ----

find_package(Python3 REQUIRED COMPONENTS Interpreter)

execute_process(COMMAND ${Python3_EXECUTABLE} -c "import pytest" RESULT_VARIABLE _pytest_found OUTPUT_QUIET ERROR_QUIET)
if(_pytest_found EQUAL 0)
  set(HAVE_PYTEST TRUE)
else()
  set(HAVE_PYTEST FALSE)
endif()


# ---- Launch Configuration ----
if (USE_CPP_CONTROLLER)
  message(STATUS "Configuration: Launcher will use the C++ Controller.")

  # Pass the backend choice and the exact path of the C++ executable to Python.
  add_custom_target(run
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/launch.py --backend cpp --cpp-exec $<TARGET_FILE:water_tank_controller>  
    COMMENT "Launching system with C++ controller..."
    VERBATIM
  )

  add_dependencies(run water_tank_controller)

else()
  message(STATUS "Configuration: Launcher will use the Python Controller.")

  add_custom_target( run 
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/launch.py --backend python
    COMMENT "Launching system with Python controller..."
    VERBATIM
  )
endif()

# ---- pytest ----
enable_testing()

if(HAVE_PYTEST)
  add_test(NAME pytest_tests
    COMMAND ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/tests
  )
else()
  message(WARNING "pytest not found in Python; 'ctest' will report no tests. Install pytest to enable test running.")
endif()