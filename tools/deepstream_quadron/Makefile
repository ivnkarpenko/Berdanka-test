DEEPSTREAM_DIR ?= $(shell \
	for d in /opt/nvidia/deepstream/deepstream-* /opt/nvidia/deepstream/deepstream; do \
		if [ -f "$$d/sources/includes/nvdsinfer_custom_impl.h" ]; then \
			echo $$d; \
			break; \
		fi; \
	done)
CUDA_DIR ?= $(shell \
	for d in /usr/local/cuda /usr/local/cuda-*; do \
		if [ -f "$$d/include/cuda_runtime_api.h" ]; then \
			echo $$d; \
			break; \
		fi; \
	done)
CXX ?= g++
TARGET := libnvdsinfer_custom_quadron.so
SRC := nvdsinfer_custom_quadron.cpp

CXXFLAGS += -std=c++14 -Wall -Wextra -O2 -fPIC
CXXFLAGS += -I$(DEEPSTREAM_DIR)/sources/includes
CXXFLAGS += -I$(CUDA_DIR)/include
LDFLAGS += -shared

.PHONY: all clean check-deepstream check-cuda

all: $(TARGET)

check-deepstream:
	@test -n "$(DEEPSTREAM_DIR)" || (echo "DeepStream headers not found. Run: make DEEPSTREAM_DIR=/opt/nvidia/deepstream/deepstream-<version>"; exit 1)
	@test -f "$(DEEPSTREAM_DIR)/sources/includes/nvdsinfer_custom_impl.h" || (echo "Missing $(DEEPSTREAM_DIR)/sources/includes/nvdsinfer_custom_impl.h"; exit 1)

check-cuda:
	@test -n "$(CUDA_DIR)" || (echo "CUDA headers not found. Install cuda-cudart-dev-11-4 or pass CUDA_DIR=/usr/local/cuda-<version>"; exit 1)
	@test -f "$(CUDA_DIR)/include/cuda_runtime_api.h" || (echo "Missing $(CUDA_DIR)/include/cuda_runtime_api.h"; exit 1)

$(TARGET): check-deepstream check-cuda $(SRC)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $(SRC)

clean:
	rm -f $(TARGET)
