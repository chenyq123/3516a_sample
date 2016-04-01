include ../Makefile.param

APP := main_detect

# target source
CPP  := arm-hisiv300-linux-g++
SRC  := $(APP).cpp 
OBJ  := $(APP).o
TARGET := $(APP)

#BASE64_LIB_DIR := /root/work/libbase64
#BASE64_LIB_SRC := $(BASE64_LIB_DIR)/base64.c
#BASE64_LIB_OBJ := $(BASE64_LIB_DIR)/base64.o

#DET_COMM_DIR := /root/3516a/Hi3516A_SDK_V1.0.4.0/mpp/sample/libdetect_teacher
#DET_COMM_OBJ := $(DET_COMM_DIR)/detect_t.o

#DB_DET_COMM_DIR :=/root/3516a/Hi3516A_SDK_V1.0.4.0/mpp/sample/libdetect_db
#DB_DET_COMM_OBJ :=$(DB_DET_COMM_DIR)/blackboard_detect.o

#DB_LIB_COMM_DIR := $(DB_DET_COMM_DIR)
#DB_LIB_COMM_OBJ := $(DB_LIB_COMM_DIR)/libdetect_t.o

#STU_LIB_COMM_DIR := /root/3516a/Hi3516A_SDK_V1.0.4.0/mpp/sample/libdetect_stu/det
#STU_LIB_COMM_SRC :=$(wildcard $(STU_LIB_COMM_DIR)/*.cpp)
#STU_LIB_COMM_OBJ :=$(STU_LIB_COMM_SRC:%.cpp=%.o)

ZK_LIB_COMM_DIR := /root/3516a/Hi3516A_SDK_V1.0.4.0/mpp/sample/libdetect_zk
ZK_LIB_COMM_SRC :=$(wildcard $(ZK_LIB_COMM_DIR)/*.cpp)
ZK_LIB_COMM_OBJ :=$(ZK_LIB_COMM_SRC:%.cpp=%.o)

#KVCONFIG_COMM_DIR := /root/3516a/Hi3516A_SDK_V1.0.4.0/mpp/sample/libdetect_stu/libkvconfig
#KVCONFIG_COMM_OBJ := $(KVCONFIG_COMM_DIR)/KVConfig.o

#CFLAGS += -I$(DET_COMM_DIR)
#CFLAGS += -I$(KVCONFIG_COMM_DIR)
#CFLAGS += -I$(DB_DET_COMM_DIR)
#CFLAGS += -I$(DB_LIB_COMM_DIR)
#CFLAGS += -I$(BASE64_LIB_DIR)
#CFLAGS += -I$(STU_LIB_COMM_DIR)
CFLAGS += -I$(ZK_LIB_COMM_DIR)

.PHONY : clean all

all: $(TARGET)

MPI_LIBS := $(REL_LIB)/libmpi.a
IVE_LIBS := $(REL_LIB)/libive.a
OPENCV_FLAGS := -lpthread -lrt `pkg-config --libs --cflags hisiv100-opencv`

$(TARGET):$(OBJ) $(ZK_LIB_COMM_OBJ)
	$(CPP) $(CFLAGS) $(OPENCV_FLAGS) $(ZK_LIB_COMM_OBJ)  $(OBJ) -lpthread  -lm  -o $(TARGET) $(MPI_LIBS) $(AUDIO_LIBA) $(SENSOR_LIBS) $(IVE_LIBS)

$(ZK_LIB_COMM_OBJ):
	@cd $(ZK_LIB_COMM_DIR);make clean;make

$(OBJ):$(SRC)
	$(CPP) $(CFLAGS) $(OPENCV_FLAGS) $(SRC) -lpthread  -lm  -c -o  $(OBJ) $(MPI_LIBS) $(AUDIO_LIBA) $(SENSOR_LIBS) $(IVE_LIBS)

#$(OBJ): $(SRC)
#	$(CPP) $(CFLAGS) $(OPENCV_FLAGS) -lpthread  -lm -c -o $@ $^ $(MPI_LIBS) $(AUDIO_LIBA) $(SENSOR_LIBS) $(SENSOR_LIBS)

#$(TARGET): $(OBJ)  $(DET_COMM_OBJ) $(KVCONFIG_COMM_OBJ) $(CJSON_COMM_OBJ) $(DB_LIB_COMM_OBJ) $(DB_DET_COMM_OBJ) $(BASE64_LIB_OBJ) $(STU_LIB_COMM_OBJ)
#	$(CPP) $(CFLAGS) $(OPENCV_FLAGS) -lpthread -lm -o $@ $^ $(MPI_LIBS) $(AUDIO_LIBA) $(SENSOR_LIBS) $(IVE_LIBS)

#$(CJSON_COMM_OBJ):$(CJSON_COMM_SRC)
#	$(CPP) -c $(CJSON_COMM_SRC) -o $(CJSON_COMM_OBJ)

#$(BASE64_LIB_OBJ):$(BASE64_LIB_SRC)
#	$(CPP) -c $(BASE64_LIB_SRC) -o $(BASE64_LIB_OBJ)

#$(STU_LIB_COMM_OBJ):%.o:%.cpp
#	$(CPP) $(CFLAGS) $(OPENCV_FLAGS) -lpthread  -lm -c -o $@ $^ $(MPI_LIBS) $(AUDIO_LIBA) $(SENSOR_LIBS) $(SENSOR_LIBS)


#LIBOBJ:
#	@cd $(KVCONFIG_COMM_DIR); make clean;make
#	@cd $(DET_COMM_DIR); make clean; make

clean:
	@rm -f $(TARGET)
	@rm -f $(OBJ)
	@rm -f $(CJSON_COMM_OBJ)
	@rm -f $(BASE64_LIB_OBJ)

copy:
	@cp ive_detect ~/hi3516c/nfs/

echo:
	@echo $(CFLAGS)
