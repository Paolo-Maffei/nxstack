PLATFORM=
TARGET	= libSSI.a
OBJECTS = libSSI.o
CFLAGS	= -Wall -D_REENTRANT -I.
LDFLAGS = -L. -D_REENTRANT -lpthread

all: $(TARGET)

$(TARGET): $(OBJECTS)
	ar rcs $(TARGET) $(OBJECTS)
	
.c.o:
	$(PLATFORM)gcc -g -c -O2 -Wall -o $(OBJECTS) $<

clean:
	rm -f $(TARGET) $(OBJECTS)
