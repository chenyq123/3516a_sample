include ../Makefile.param

# target source
CPP  := arm-hisiv300-linux-g++
SRC  := detect.cpp 
OBJ  := detect.o
TARGET := detect

DET_COMM_DIR := /root/3516a/Hi3516A_SDK_V1.0.4.0/mpp/sample/libdetect3
DET_COMM_OBJ := $(DET_COMM_DIR)/detect_t.o

KVCONFIG_COMM_DIR := /root/3516a/Hi3516A_SDK_V1.0.4.0/mpp/sample/libdetect2/libkvconfig
KVCONFIG_COMM_OBJ := $(KVCONFIG_COMM_DIR)/KVConfig.o

CFLAGS += -I$(DET_COMM_DIR)
CFLAGS += -I$(KVCONFIG_COMM_DIR)

.PHONY : clean all

all: LIBOBJ $(CJSON_COMM_OBJ) $(TARGET) $(OBJ) 

MPI_LIBS := $(REL_LIB)/libmpi.a
IVE_LIBS := $(REL_LIB)/libive.a
OPENCV_FLAGS := -lpthread -lrt `pkg-config --libs --cflags hisiv100-opencv`

$(OBJ): $(SRC)
	$(CPP) $(CFLAGS) $(OPENCV_FLAGS) -lpthread  -lm -c -o $@ $^ $(MPI_LIBS) $(AUDIO_LIBA) $(SENSOR_LIBS) $(SENSOR_LIBS)

$(TARGET): $(OBJ)  $(DET_COMM_OBJ) $(KVCONFIG_COMM_OBJ) $(CJSON_COMM_OBJ) $(COMM_OBJ)
	$(CPP) $(CFLAGS) $(OPENCV_FLAGS) -lpthread -lm -o $@ $^ $(MPI_LIBS) $(AUDIO_LIBA) $(SENSOR_LIBS) $(IVE_LIBS)

$(CJSON_COMM_OBJ):$(CJSON_COMM_SRC)
	$(CPP) -c $(CJSON_COMM_SRC) -o $(CJSON_COMM_OBJ)

LIBOBJ:
	@cd $(KVCONFIG_COMM_DIR); make clean;make
	@cd $(DET_COMM_DIR); make clean; make

clean:
	@rm -f $(TARGET)
	@rm -f $(OBJ)
	@rm -f $(CJSON_COMM_OBJ)

copy:
	@cp ive_detect ~/hi3516c/nfs/

echo:
	@echo $(DET_COMM_OBJ)
