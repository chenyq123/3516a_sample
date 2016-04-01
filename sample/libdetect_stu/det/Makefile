
OPENCV_FLAGS := -lpthread -lrt `pkg-config --libs --cflags hisiv100-opencv`
CC := arm-hisiv300-linux-g++

SRC = $(wildcard *.cpp)
OBJ = $(SRC:%.cpp=%.o)
INC_PATH := /root/3516a/Hi3516A_SDK_V1.0.4.0/mpp/include/
REL_LIB := /root/3516a/Hi3516A_SDK_V1.0.4.0/mpp/lib
MPI_LIBS := $(REL_LIB)/libmpi.a
IVE_LIBS := $(REL_LIB)/libive.a
CFLAGS := -I $(INC_PATH)
all: $(OBJ)

$(OBJ): %.o: %.cpp
	$(CC) $(OPENCV_FLAGS) $(CFLAGS) $(MPI_LIBS) $(IVE_LIBS) -pthread -lm -c $< -o $@

clean:
	rm *.o
