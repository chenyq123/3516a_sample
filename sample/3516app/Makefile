
include ../Makefile.param
#SRC  := $(wildcard *.c) 
#OBJ  := $(SRC:%.c=%.o)

SRC := 3516a.c
OBJ := $(SRC:%.c=%.o)

TEST_COMM_PATH := $(MPP_PATH)/sample/test_comm
TEST_COMM_SRC := $(wildcard $(TEST_COMM_PATH)/*.c)
TEST_COMM_OBJ := $(TEST_COMM_SRC:%.c=%.o)

CFLAGS += -I $(TEST_COMM_PATH)

TARGET := $(OBJ:%.o=%)
.PHONY : clean all

all: $(TARGET) $(OBJ)

MPI_LIBS := $(REL_LIB)/libmpi.a
IVE_LIBS := $(REL_LIB)/libive.a
OPENCV_FLAGS := -lpthread -lrt `pkg-config --libs --cflags hisiv100-opencv`

$(OBJ): $(SRC)
	$(CC) $(CFLAGS)  -lpthread -lmath -lm -c -o $@ $^ $(MPI_LIBS) $(AUDIO_LIBA) $(SENSOR_LIBS)  -lstdc++

$(TARGET): $(OBJ) $(COMM_OBJ) $(TEST_COMM_OBJ)
	$(CC) $(CFLAGS)  -lpthread -lm -o $@ $^ $(MPI_LIBS) $(AUDIO_LIBA) $(SENSOR_LIBS) $(IVE_LIBS) -lstdc++

clean:
	@rm -f $(TARGET)
	@rm -f $(OBJ)
	@rm -f $(COMM_OBJ)

echo:
	@echo $()
