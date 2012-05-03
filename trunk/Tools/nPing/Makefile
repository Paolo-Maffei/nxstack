PLATFORM=
TARGET	= nPing
OBJECTS = nPing.o
CFLAGS	= -Wall -D_REENTRANT -I.
LDFLAGS = -D_REENTRANT -L../libnRP/ -lnRP

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(PLATFORM)gcc -o $(TARGET) $(OBJECTS) $(LDFLAGS)

.c.o:
	$(PLATFORM)gcc -c -O2 $(CFLAGS) $< -o $(OBJECTS)

clean:
	rm -f $(TARGET) $(OBJECTS)
