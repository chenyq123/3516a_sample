# Hisilicon Hi3516 sample Makefile

include ../Makefile.param
#ifeq ($(SAMPLE_PARAM_FILE), )
#     SAMPLE_PARAM_FILE:=../Makefile.param
#     include $(SAMPLE_PARAM_FILE)
#endif

# target source
CPP  := arm-hisiv300-linux-g++
SRC  := $(wildcard *.c) 
OBJ  := $(SRC:%.c=%.o)

TARGET := $(OBJ:%.o=%)
.PHONY : clean all

all: $(TARGET) $(OBJ) LIBOBJ

MPI_LIBS := $(REL_LIB)/libmpi.a
IVE_LIBS := $(REL_LIB)/libive.a
OPENCV_FLAGS := -lpthread -lrt `pkg-config --libs --cflags hisiv100-opencv`

$(OBJ): %.o: %.c
	$(CPP) $(CFLAGS) $(OPENCV_FLAGS) -lpthread -lmath -lm -c -o $@ $^ $(MPI_LIBS) $(AUDIO_LIBA) $(SENSOR_LIBS)  

$(TARGET):%:%.o $(COMM_OBJ) $(TEST_COMM_OBJ)  $(DET_COMM_OBJ) $(KVCONFIG_COMM_OBJ)
	$(CPP) $(CFLAGS) $(OPENCV_FLAGS) -lpthread -lm -o $@ $^ $(MPI_LIBS) $(AUDIO_LIBA) $(SENSOR_LIBS) $(IVE_LIBS) 

LIBOBJ:
	@cd $(KVCONFIG_COMM_DIR); make clean;make
	@cd $(DET_COMM_DIR); make clean; make

clean:
	@rm -f $(TARGET)
	@rm -f $(OBJ)

copy:
	@cp ive_detect ~/hi3516c/nfs/

echo:
	@echo $(DET_COMM_OBJ)
