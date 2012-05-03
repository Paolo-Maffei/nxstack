PLATFORM=
TARGET	= nRouted
OBJECTS = nRouted.o tcpserver.o serialserver.o port.o
CFLAGS	= -Wall -D_REENTRANT -I.
LDFLAGS = -L. -D_REENTRANT -lpthread

all: $(TARGET) tools

$(TARGET): $(OBJECTS)
	$(PLATFORM)gcc -g -o $(TARGET) $(OBJECTS) $(LDFLAGS)
	
.c.o:
	$(PLATFORM)gcc -c -g -O2 -Wall $(CFLAGS) $<

tools:
	for i in $(SUBDIRS); do make -C $$i; done

clean:
	rm -f $(TARGET) $(OBJECTS)
