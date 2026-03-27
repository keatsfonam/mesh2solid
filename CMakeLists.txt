cmake_minimum_required(VERSION 3.20)
project(mesh2solid LANGUAGES CXX)

option(MESH2SOLID_ENABLE_CGAL "Enable CGAL when available" ON)
option(MESH2SOLID_ENABLE_OPENCASCADE "Enable Open CASCADE when available" ON)

# Keep the CMake build profile aligned with the Makefile's O2 setting.
# This avoids OOM kills in the Docker full-stack CGAL/OCCT build that can occur at O3.
foreach(config RELEASE RELWITHDEBINFO)
  if(DEFINED CMAKE_CXX_FLAGS_${config})
    string(REPLACE "-O3" "-O2" CMAKE_CXX_FLAGS_${config} "${CMAKE_CXX_FLAGS_${config}}")
  endif()
endforeach()

find_package(ZLIB REQUIRED)

set(MESH2SOLID_PIPELINE_IMPL
  src/pipeline/io.inc
  src/pipeline/cgal.inc
  src/pipeline/reconstruction.inc
  src/pipeline/fallback.inc
  src/pipeline/output.inc
)

add_executable(mesh2solid
  src/main.cpp
  src/pipeline.cpp
  ${MESH2SOLID_PIPELINE_IMPL}
)

target_include_directories(mesh2solid PRIVATE include)
target_compile_features(mesh2solid PRIVATE cxx_std_20)
target_compile_options(mesh2solid PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(mesh2solid PRIVATE ZLIB::ZLIB)

if(MESH2SOLID_ENABLE_CGAL)
  find_package(CGAL QUIET)
  if(CGAL_FOUND)
    target_compile_definitions(mesh2solid PRIVATE MESH2SOLID_WITH_CGAL=1)
    target_link_libraries(mesh2solid PRIVATE CGAL::CGAL)
  endif()
endif()

if(MESH2SOLID_ENABLE_OPENCASCADE)
  find_package(OpenCASCADE QUIET COMPONENTS
    FoundationClasses
    ModelingData
    ModelingAlgorithms
    Visualization
    ApplicationFramework
    DataExchange
  )
  if(OpenCASCADE_FOUND)
    target_compile_definitions(mesh2solid PRIVATE MESH2SOLID_WITH_OCCT=1)
    target_include_directories(mesh2solid PRIVATE ${OpenCASCADE_INCLUDE_DIR})

    set(MESH2SOLID_OCCT_LIBRARIES
      ${OpenCASCADE_FoundationClasses_LIBRARIES}
      ${OpenCASCADE_ModelingData_LIBRARIES}
      ${OpenCASCADE_ModelingAlgorithms_LIBRARIES}
      TKXSBase
      TKSTEPBase
      TKSTEPAttr
      TKSTEP209
      TKSTEP
    )
    list(REMOVE_DUPLICATES MESH2SOLID_OCCT_LIBRARIES)
    target_link_libraries(mesh2solid PRIVATE ${MESH2SOLID_OCCT_LIBRARIES})
  endif()
endif()
