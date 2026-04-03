project(
  chainmaker
  VERSION 0.1.0
  DESCRIPTION "ChainMaker - Multi-chain bead-on-string robot designer"
)

enable_language(C)
enable_language(CXX)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

set(MUJOCO_SAMPLE_COMPILE_OPTIONS "${AVX_COMPILE_OPTIONS}" "${EXTRA_COMPILE_OPTIONS}")
set(MUJOCO_SAMPLE_LINK_OPTIONS "${EXTRA_LINK_OPTIONS}")

if(MUJOCO_HARDEN)
  if(WIN32)
    set(MUJOCO_SAMPLE_LINK_OPTIONS "${MUJOCO_SAMPLE_LINK_OPTIONS}" -Wl,/DYNAMICBASE)
  else()
    set(MUJOCO_SAMPLE_COMPILE_OPTIONS "${MUJOCO_SAMPLE_COMPILE_OPTIONS}" -fPIE)
    if(APPLE)
      set(MUJOCO_SAMPLE_LINK_OPTIONS "${MUJOCO_SAMPLE_LINK_OPTIONS}" -Wl,-pie)
    else()
      set(MUJOCO_SAMPLE_LINK_OPTIONS "${MUJOCO_SAMPLE_LINK_OPTIONS}" -pie)
    endif()
  endif()
endif()

# Fetch nlohmann/json for JSON save/load
include(FetchContent)
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG        v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

add_executable(ChainMaker
  main.cc
  chainmaker.h
  chainmaker.cpp
  chainmaker_render.h
  chainmaker_render.cpp
  chainmaker_ui.h
  chainmaker_ui.cpp
  chainmaker_io.h
  chainmaker_io.cpp
  chainmaker_compile.h
  chainmaker_compile.cpp
  chainmaker_sim.h
  chainmaker_sim.cpp
  chainmaker_ipc.h
  chainmaker_ipc.cpp
)

target_compile_options(ChainMaker PUBLIC ${MUJOCO_SAMPLE_COMPILE_OPTIONS})

find_package(Threads REQUIRED)
target_link_libraries(
  ChainMaker
  mujoco::mujoco
  glfw
  Threads::Threads
  nlohmann_json::nlohmann_json
)

# Windows sockets (needed by chainmaker_ipc.cpp)
if(WIN32)
  target_link_libraries(ChainMaker ws2_32)
endif()

target_link_options(ChainMaker PRIVATE ${MUJOCO_SAMPLE_LINK_OPTIONS})

# Set ChainMaker as the default startup project in Visual Studio
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ChainMaker)
