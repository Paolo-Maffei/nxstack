PLATFORM=
TARGET	= SSI-Browser
OBJECTS = SSI-Browser.o
CFLAGS	= -Wall -D_REENTRANT -I.
LDFLAGS = -D_REENTRANT -lpthread -lm -L../libnRP/ -lnRP -L../libSSI -lSSI

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(PLATFORM)gcc -o $(TARGET) $(OBJECTS) $(LDFLAGS)
	
.c.o:
	$(PLATFORM)gcc -c -O2 -Wall $<

clean:
	rm -f $(TARGET) $(OBJECTS)
