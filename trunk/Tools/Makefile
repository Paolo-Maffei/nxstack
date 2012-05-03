TOOL_DIRS = `find . -maxdepth 1 -type d -name "[!.]*" | grep -v lib`
LIB_DIRS = libnRP
LIB_DIRS += `find . -maxdepth 1 -type d -name "[!.]*" | grep lib | grep -v libnRP`
SUBDIRS = $(LIB_DIRS) $(TOOL_DIRS)

all:
	@echo $(SUBDIRS)
	for i in $(SUBDIRS); do \
		cd $$i; make; cd ..; done

clean:
	@echo $(SUBDIRS)
	for i in $(SUBDIRS); do \
		cd $$i; make clean ; cd ..; done

clean-deps:
	@echo "Clean complete."
	
show-tools:
	@echo -n "Available tools: "
	@echo $(SUBDIRS)

	