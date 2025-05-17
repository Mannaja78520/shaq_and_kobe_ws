# CXX = g++
# CXXFLAGS = -std=c++17 `pkg-config --cflags opencv4` -I/usr/local/include
# # LDFLAGS = `pkg-config --libs opencv4` -L/usr/local/lib -ltensorflowlite
# LDFLAGS = `pkg-config --libs opencv4` -L/usr/local/lib

# TARGET = test_hoop_detect_onnx
# SRC = test_hoop_detect_onnx.cpp

# # Preserve your existing LD_LIBRARY_PATH, don't override to empty
# export LD_LIBRARY_PATH := /lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/usr/local/lib:$(LD_LIBRARY_PATH)

# all:
# 	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

# run: all
# 	LD_LIBRARY_PATH=$(LD_LIBRARY_PATH) ./$(TARGET)

# clean:
# 	rm -f $(TARGET) *.o


all:
	g++ test_hoop_detect_onnx.cpp -o test_hoop_detect_onnx \
		`pkg-config --cflags --libs opencv4` \
		-std=c++17 \
		-lonnxruntime

	./test_hoop_detect_onnx
	rm -rf test_hoop_detect_onnx