CXX ?= clang++
CXXFLAGS ?= -std=c++20 -O2 -Wall -Wextra -pedantic -Iinclude
LDFLAGS ?= -lz

SRC := src/main.cpp src/pipeline.cpp
PIPELINE_IMPL := src/pipeline/io.inc src/pipeline/cgal.inc src/pipeline/reconstruction.inc src/pipeline/fallback.inc src/pipeline/output.inc
TARGET := build/mesh2solid
DOCKER_IMAGE ?= mesh2solid-dev:latest
DOCKER_RUN := docker run --rm -u $$(id -u):$$(id -g) -v "$(CURDIR)":/workspace -w /workspace $(DOCKER_IMAGE)

.PHONY: all clean test golden hard-bench smoke-bench cmake-minimal docker-image docker-build docker-test docker-hard-bench docker-shell

all: $(TARGET)

$(TARGET): $(SRC) $(PIPELINE_IMPL) include/mesh2solid/pipeline.h | build
	$(CXX) $(CXXFLAGS) $(SRC) $(LDFLAGS) -o $(TARGET)

build:
	mkdir -p build

test: $(TARGET)
	python3 -m unittest discover -s tests -p 'test_*.py'

golden: $(TARGET)
	python3 tests/update_goldens.py

hard-bench: docker-image
	rm -rf build-cmake/docker-full
	mkdir -p build-cmake
	$(DOCKER_RUN) bash -lc "cmake -S . -B build-cmake/docker-full -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DMESH2SOLID_ENABLE_CGAL=ON -DMESH2SOLID_ENABLE_OPENCASCADE=ON && cmake --build build-cmake/docker-full --parallel 1 && MESH2SOLID_BIN=build-cmake/docker-full/mesh2solid MESH2SOLID_SKIP_BUILD=1 python3 benchmarks/run_examples.py --expectation-profile docker-full"

smoke-bench: $(TARGET)
	python3 benchmarks/run_examples.py --expectation-profile host-minimal

cmake-minimal:
	mkdir -p build-cmake
	cmake --preset host-minimal
	cmake --build --preset host-minimal

docker-image:
	docker build -t $(DOCKER_IMAGE) .

docker-build: docker-image
	rm -rf build-cmake/docker-full
	mkdir -p build-cmake
	$(DOCKER_RUN) bash -lc "cmake -S . -B build-cmake/docker-full -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DMESH2SOLID_ENABLE_CGAL=ON -DMESH2SOLID_ENABLE_OPENCASCADE=ON && cmake --build build-cmake/docker-full --parallel 1"

docker-test: docker-image
	rm -rf build-cmake/docker-full
	mkdir -p build-cmake
	$(DOCKER_RUN) bash -lc "cmake -S . -B build-cmake/docker-full -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DMESH2SOLID_ENABLE_CGAL=ON -DMESH2SOLID_ENABLE_OPENCASCADE=ON && cmake --build build-cmake/docker-full --parallel 1 && MESH2SOLID_BIN=build-cmake/docker-full/mesh2solid MESH2SOLID_SKIP_BUILD=1 python3 -m unittest discover -s tests -p 'test_*.py'"

docker-hard-bench: hard-bench

docker-shell: docker-image
	docker run --rm -it -u $$(id -u):$$(id -g) -v "$(CURDIR)":/workspace -w /workspace $(DOCKER_IMAGE) bash

clean:
	rm -rf build build-cmake
