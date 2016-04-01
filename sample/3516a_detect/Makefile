# Hisilicon Hi3516 sample Makefile

include ../Makefile.param
#ifeq ($(SAMPLE_PARAM_FILE), )
#     SAMPLE_PARAM_FILE:=../Makefile.param
#     include $(SAMPLE_PARAM_FILE)
#endif

# target source
CPP  := arm-hisiv300-linux-gcc
SRC  := $(wildcard *.c) 
OBJ  := $(SRC:%.c=%.o)
CONF_FILE_DIR := /root/conf_file_comm
CONF_FILE_SRC := $(CONF_FILE_DIR)/conf_file.c
CONF_FILE_OBJ := $(CONF_FILE_DIR)/conf_file.o

TARGET := $(OBJ:%.o=%)
.PHONY : clean all

all: $(TARGET) $(OBJ) $(CJSON_COMM_OBJ) $(CONF_FILE_OBJ)

MPI_LIBS := $(REL_LIB)/libmpi.a
IVE_LIBS := $(REL_LIB)/libive.a
CFLAGS += -I$(CONF_FILE_DIR)
#OPENCV_FLAGS := -lpthread -lrt `pkg-config --libs --cflags hisiv100-opencv`

$(OBJ): %.o: %.c
	$(CPP) $(CFLAGS)  -lpthread -lmath -lm -c -o $@ $^ $(MPI_LIBS) $(AUDIO_LIBA) $(SENSOR_LIBS)

$(TARGET):%:%.o $(COMM_OBJ) $(TEST_COMM_OBJ)  $(CJSON_COMM_OBJ) $(CONF_FILE_OBJ)
	$(CPP) $(CFLAGS)  -lpthread -lm -o $@ $^ $(MPI_LIBS) $(AUDIO_LIBA) $(SENSOR_LIBS) $(IVE_LIBS) 

$(CJSON_COMM_OBJ):$(CJSON_COMM_SRC)
	$(CPP) -c $(CJSON_COMM_SRC) -o $(CJSON_COMM_OBJ)

$(CONF_FILE_OBJ):$(CONF_FILE_SRC)
	$(CPP) -c $(CONF_FILE_SRC) -o $(CONF_FILE_OBJ)


clean:
	@rm -f $(TARGET)
	@rm -f $(OBJ)
	@rm -f $(COMM_OBJ)
	@rm -f $(CJSON_COMM_OBJ)
	@rm -f $(CONF_FILE_OBJ)

cleanstream:
	@rm -f *.h264
	@rm -f *.jpg
	@rm -f *.mjp
	@rm -f *.mp4

copy:
	@cp ive_detect ~/hi3516c/nfs/
