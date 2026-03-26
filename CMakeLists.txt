cmake_minimum_required(VERSION 3.20)
project(stl2solid LANGUAGES CXX)

option(STL2SOLID_ENABLE_CGAL "Enable CGAL when available" ON)
option(STL2SOLID_ENABLE_OPENCASCADE "Enable Open CASCADE when available" ON)

add_executable(stl2solid
  src/main.cpp
  src/pipeline.cpp
)

target_include_directories(stl2solid PRIVATE include)
target_compile_features(stl2solid PRIVATE cxx_std_20)
target_compile_options(stl2solid PRIVATE -Wall -Wextra -Wpedantic)

if(STL2SOLID_ENABLE_CGAL)
  find_package(CGAL QUIET)
  if(CGAL_FOUND)
    target_compile_definitions(stl2solid PRIVATE STL2SOLID_WITH_CGAL=1)
    target_link_libraries(stl2solid PRIVATE CGAL::CGAL)
  endif()
endif()

if(STL2SOLID_ENABLE_OPENCASCADE)
  find_package(OpenCASCADE QUIET)
  if(OpenCASCADE_FOUND)
    target_compile_definitions(stl2solid PRIVATE STL2SOLID_WITH_OCCT=1)
    target_include_directories(stl2solid PRIVATE ${OpenCASCADE_INCLUDE_DIR})
    target_link_libraries(stl2solid PRIVATE ${OpenCASCADE_LIBRARIES})
  endif()
endif()

