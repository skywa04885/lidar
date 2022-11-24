CPP_SOURCES += $(shell find . -name *.cpp)
C_SOURCES += $(shell find . -name *.c)

OBJECTS += $(CPP_SOURCES:.cpp=.o)
OBJECTS += $(C_SOURCES:.c=.o)

CPP_INCLUDES += -I./inc
CPP_INCLUDES += -I./vendor/sl_lidar_sdk/inc

C_INCLUDES += -I./vendor/sl_lidar_sdk/inc

CPP_LINKER_ARGS += -L./libraries

%.o: %.cpp
	g++ -std=c++20 $(CPP_INCLUDES) $< -c -o $@
%.o: %.c
	gcc $(C_INCLUDES) $< -c -o $@

all: $(OBJECTS)
	g++ -std=c++20 $(CPP_LINKER_ARGS) $(OBJECTS)
clean:
	rm -rf $(OBJECTS)