CPP := arm-hisiv300-linux-g++
OPENCV_FLAGS := -lpthread -lrt `pkg-config --libs --cflags hisiv100-opencv`
KVCONFIG_COMM_DIR :=/root/3516a/Hi3516A_SDK_V1.0.4.0/mpp/sample/libdetect2/libkvconfig
DET_COMM_DIR :=/root/3516a/Hi3516A_SDK_V1.0.4.0/mpp/sample/libdetect2/det
INC_FLAGS := -I$(DET_COMM_DIR)
INC_FLAGS += -I$(KVCONFIG_COMM_DIR)

APP := detect_face

KVCONFIG_OBJ := $(KVCONFIG_COMM_DIR)/KVConfig.o
OBJDET_OBJ := $(DET_COMM_DIR)/objdet.o

all: $(APP).o OBJ
	$(CPP) $(OPENCV_FLAGS) $(INC_FLAGS) $(APP).o $(KVCONFIG_OBJ) $(OBJDET_OBJ) -o $(APP)

$(APP).o:$(APP).cpp
	$(CPP) $(OPENCV_FLAGS) $(INC_FLAGS) -c $(APP).cpp -o $(APP).o

OBJ:
	@cd $(KVCONFIG_COMM_DIR); make clean; make
	@cd $(DET_COMM_DIR);make clean; make

clean:
	@rm *.o $(APP)
