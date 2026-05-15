DEEPSTREAM_DIR ?= /opt/nvidia/deepstream/deepstream
CXX ?= g++
TARGET := libnvdsinfer_custom_quadron.so
SRC := nvdsinfer_custom_quadron.cpp

CXXFLAGS += -std=c++14 -Wall -Wextra -O2 -fPIC
CXXFLAGS += -I$(DEEPSTREAM_DIR)/sources/includes
LDFLAGS += -shared

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $<

clean:
	rm -f $(TARGET)
