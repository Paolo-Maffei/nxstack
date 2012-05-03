SUBDIRS = `find . -maxdepth 1 -type d -name "[!.]*"`

all:
	@echo $(SUBDIRS)
	for i in $(SUBDIRS); do \
		cd $$i; make; cd ..; done

clean:
	@echo $(SUBDIRS)
	for i in $(SUBDIRS); do \
		cd $$i; make clean ; make clean-deps; cd ..; done

show-examples:
	@echo -n "Available examples: "
	@echo $(SUBDIRS)

