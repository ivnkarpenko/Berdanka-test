DEEPSTREAM_DIR ?= $(shell \
	for d in /opt/nvidia/deepstream/deepstream-* /opt/nvidia/deepstream/deepstream; do \
		if [ -f "$$d/sources/includes/nvdsinfer_custom_impl.h" ]; then \
			echo $$d; \
			break; \
		fi; \
	done)
CXX ?= g++
TARGET := libnvdsinfer_custom_quadron.so
SRC := nvdsinfer_custom_quadron.cpp

CXXFLAGS += -std=c++14 -Wall -Wextra -O2 -fPIC
CXXFLAGS += -I$(DEEPSTREAM_DIR)/sources/includes
LDFLAGS += -shared

.PHONY: all clean

all: $(TARGET)

check-deepstream:
	@test -n "$(DEEPSTREAM_DIR)" || (echo "DeepStream headers not found. Run: make DEEPSTREAM_DIR=/opt/nvidia/deepstream/deepstream-<version>"; exit 1)
	@test -f "$(DEEPSTREAM_DIR)/sources/includes/nvdsinfer_custom_impl.h" || (echo "Missing $(DEEPSTREAM_DIR)/sources/includes/nvdsinfer_custom_impl.h"; exit 1)

$(TARGET): check-deepstream $(SRC)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $<

clean:
	rm -f $(TARGET)
