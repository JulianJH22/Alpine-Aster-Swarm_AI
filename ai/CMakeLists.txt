cmake_minimum_required(VERSION 3.16)
project(IAEnjambre VERSION 0.1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_library(swarm_core src/core.cpp)
target_include_directories(swarm_core PUBLIC include)
target_compile_options(swarm_core PRIVATE -Wall -Wextra -Wpedantic)
add_executable(swarm_demo src/main.cpp)
target_link_libraries(swarm_demo PRIVATE swarm_core)
enable_testing()
add_executable(swarm_tests tests/test_core.cpp)
target_link_libraries(swarm_tests PRIVATE swarm_core)
add_test(NAME swarm_tests COMMAND swarm_tests)

# Tests del gemelo digital (Python/PyBullet, Fase 4). Requiere: pip install -r sim/requirements.txt
option(SWARM_SIM_TESTS "Registrar los tests Python de sim/ en CTest" OFF)
if(SWARM_SIM_TESTS)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME swarm_sim_tests
           COMMAND ${Python3_EXECUTABLE} -m pytest tests -q
           WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/sim)
endif()

# Optional C++ TCP/JSON adapter to the Python Fase 4 simulator.
# Only compiled when json-c is available; the core remains dependency-free.
option(SWARM_GATEWAY "Build POSIX simulator gateway adapter" ON)
if(SWARM_GATEWAY AND UNIX)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(JSONC QUIET IMPORTED_TARGET json-c)
  endif()
  if(TARGET PkgConfig::JSONC)
    add_library(swarm_gateway src/gateway.cpp)
    target_include_directories(swarm_gateway PUBLIC include)
    target_link_libraries(swarm_gateway PUBLIC swarm_core PkgConfig::JSONC)
    target_compile_options(swarm_gateway PRIVATE -Wall -Wextra -Wpedantic)
    add_executable(swarm_gateway_demo src/gateway_demo.cpp)
    target_link_libraries(swarm_gateway_demo PRIVATE swarm_gateway)
  else()
    message(STATUS "json-c not found: gateway disabled; swarm_core still builds")
  endif()
endif()
if(TARGET swarm_gateway_demo)
  find_package(Python3 QUIET COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    add_test(NAME gateway_contract
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_gateway_contract.py $<TARGET_FILE:swarm_gateway_demo>)
  endif()
endif()

# Integracion de controlador C++ de Fase 3.5.1 en Fase 4.
# Mantiene el gateway TCP/JSON y el simulador de Fase 4 sin modificaciones.
if(TARGET swarm_gateway)
  add_executable(swarm_control_demo src/gateway_control_demo.cpp)
  target_link_libraries(swarm_control_demo PRIVATE swarm_gateway)
  target_compile_options(swarm_control_demo PRIVATE -Wall -Wextra -Wpedantic)
  if(Python3_Interpreter_FOUND)
    add_test(NAME gateway_control_contract
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_gateway_control.py $<TARGET_FILE:swarm_control_demo> ${CMAKE_CURRENT_SOURCE_DIR}/config/control_demo.json)
  endif()
endif()

# Phase 5.1: diagnostic client, no learning or MPC training.
if(TARGET swarm_gateway)
  add_executable(swarm_phase5_cli src/gateway_phase5_cli.cpp)
  target_link_libraries(swarm_phase5_cli PRIVATE swarm_gateway)
endif()
