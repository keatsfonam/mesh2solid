CXX ?= clang++
CXXFLAGS ?= -std=c++20 -O2 -Wall -Wextra -pedantic -Iinclude
LDFLAGS ?= -lz

SRC := src/main.cpp src/pipeline.cpp
TARGET := build/mesh2solid

.PHONY: all clean test golden

all: $(TARGET)

$(TARGET): $(SRC) include/mesh2solid/pipeline.h | build
	$(CXX) $(CXXFLAGS) $(SRC) $(LDFLAGS) -o $(TARGET)

build:
	mkdir -p build

test: $(TARGET)
	python3 -m unittest discover -s tests -p 'test_*.py'

golden: $(TARGET)
	python3 tests/update_goldens.py

clean:
	rm -rf build
