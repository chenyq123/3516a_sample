OPENCV_FLAGS := -lpthread -lrt `pkg-config --libs --cflags hisiv100-opencv`
CC := arm-hisiv300-linux-g++

SRC = $(wildcard *.cpp)
OBJ = $(SRC:%.cpp=%.o)
all: $(OBJ)
	
$(OBJ): %.o: %.cpp
	$(CC) $(OPENCV_FLAGS) -c  $< -o $@


clean:
	rm *.o
