cmake_minimum_required(VERSION 3.20)
project(mesh2solid LANGUAGES CXX)

option(MESH2SOLID_ENABLE_CGAL "Enable CGAL when available" ON)
option(MESH2SOLID_ENABLE_OPENCASCADE "Enable Open CASCADE when available" ON)

find_package(ZLIB REQUIRED)

set(MESH2SOLID_PIPELINE_IMPL
  src/pipeline/io.inc
  src/pipeline/reconstruction.inc
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
  find_package(OpenCASCADE QUIET)
  if(OpenCASCADE_FOUND)
    target_compile_definitions(mesh2solid PRIVATE MESH2SOLID_WITH_OCCT=1)
    target_include_directories(mesh2solid PRIVATE ${OpenCASCADE_INCLUDE_DIR})
    target_link_libraries(mesh2solid PRIVATE ${OpenCASCADE_LIBRARIES})
  endif()
endif()
