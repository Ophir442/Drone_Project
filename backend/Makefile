CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -Iinclude -Ithird_party -pthread
SRC      := $(wildcard src/*.cpp)
TARGET   := bread_delivery

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@

run: $(TARGET)
	./$(TARGET) config.json

server: $(TARGET)
	./$(TARGET) config.json --server

clean:
	rm -f $(TARGET)

.PHONY: all run server clean
