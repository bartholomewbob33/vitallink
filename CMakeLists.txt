# -----------------------------------------------------------------------------
# File: CMakeLists.txt
# Purpose: Full CMake setup for the VitalLink wristband firmware.
# -----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.20.0)

# Zephyr + project
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(vitalink_band)

# Public headers
target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Devicetree overlay (auto-pick if present)
set(_OVERLAY_PATH ${CMAKE_CURRENT_SOURCE_DIR}/boards/nrf5340dk_nrf5340_cpuapp.overlay)
if(EXISTS ${_OVERLAY_PATH})
  set(DTC_OVERLAY_FILE ${_OVERLAY_PATH} CACHE STRING "" FORCE)
endif()

# Sources (raw I2C paths; no ppg_engine.c)
target_sources(app PRIVATE
  src/main.c
  src/sensors_vitalink.c
  src/pmic_bq25155.c
)

# Math lib for sqrt() used in sensors_vitalink.c
target_link_libraries(app PRIVATE m)
