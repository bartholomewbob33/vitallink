# -----------------------------------------------------------------------------
# File: CMakeLists.txt
# Purpose: Minimal CMake setup for a Zephyr/nRF Connect SDK application.
# - Finds Zephyr (from the toolchain you opened via Toolchain Manager).
# - Declares this app ("vitalink_band").
# - Adds our src/ and include/ folders to the build.
# -----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.20.0)

# Find Zephyr using the environment that VS Code opened for NCS v2.9.2
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(vitalink_band)

# Make headers in include/ visible to our source files
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Compile our app code
target_sources(app PRIVATE src/main.c)
