OPENCV_FLAGS := -lpthread -lrt `pkg-config --libs --cflags hisiv100-opencv`
CC := arm-hisiv300-linux-gcc

SRC = $(wildcard *.c)
OBJ = $(SRC:%.c=%.o)
all: $(OBJ)
	
#$(OBJ): %.o: %.c
#	$(CC) $(OPENCV_FLAGS) -c -O3 $< -o $@
$(OBJ): %.o: %.c
	$(CC)  -c -O3 $< -o $@


clean:
	rm *.o
