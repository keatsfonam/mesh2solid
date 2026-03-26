CXX ?= clang++
CXXFLAGS ?= -std=c++20 -O2 -Wall -Wextra -pedantic -Iinclude
LDFLAGS ?= -lz

SRC := src/main.cpp src/pipeline.cpp
PIPELINE_IMPL := src/pipeline/io.inc src/pipeline/cgal.inc src/pipeline/reconstruction.inc src/pipeline/fallback.inc src/pipeline/output.inc
TARGET := build/mesh2solid
DOCKER_IMAGE ?= mesh2solid-dev:latest
DOCKER_RUN := docker run --rm -u $$(id -u):$$(id -g) -v "$(CURDIR)":/workspace -w /workspace $(DOCKER_IMAGE)

.PHONY: all clean test golden hard-bench cmake-minimal docker-image docker-build docker-test docker-hard-bench docker-shell

all: $(TARGET)

$(TARGET): $(SRC) $(PIPELINE_IMPL) include/mesh2solid/pipeline.h | build
	$(CXX) $(CXXFLAGS) $(SRC) $(LDFLAGS) -o $(TARGET)

build:
	mkdir -p build

test: $(TARGET)
	python3 -m unittest discover -s tests -p 'test_*.py'

golden: $(TARGET)
	python3 tests/update_goldens.py

hard-bench: $(TARGET)
	python3 benchmarks/run_examples.py --expectation-profile host-minimal

cmake-minimal:
	cmake --preset host-minimal
	cmake --build --preset host-minimal

docker-image:
	docker build -t $(DOCKER_IMAGE) .

docker-build: docker-image
	rm -rf build-cmake/docker-full
	$(DOCKER_RUN) bash -lc "cmake --preset docker-full && cmake --build --preset docker-full"

docker-test: docker-image
	rm -rf build-cmake/docker-full
	$(DOCKER_RUN) bash -lc "cmake --preset docker-full && cmake --build --preset docker-full && MESH2SOLID_BIN=build-cmake/docker-full/mesh2solid MESH2SOLID_SKIP_BUILD=1 python3 -m unittest discover -s tests -p 'test_*.py'"

docker-hard-bench: docker-image
	rm -rf build-cmake/docker-full
	$(DOCKER_RUN) bash -lc "cmake --preset docker-full && cmake --build --preset docker-full && MESH2SOLID_BIN=build-cmake/docker-full/mesh2solid MESH2SOLID_SKIP_BUILD=1 python3 benchmarks/run_examples.py --expectation-profile docker-full"

docker-shell: docker-image
	docker run --rm -it -u $$(id -u):$$(id -g) -v "$(CURDIR)":/workspace -w /workspace $(DOCKER_IMAGE) bash

clean:
	rm -rf build build-cmake
