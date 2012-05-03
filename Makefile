SUBDIRS = Tools Examples

all:
	@echo $(SUBDIRS)
	for i in $(SUBDIRS); do \
		cd $$i; make; cd ..; done

clean:
	@echo $(SUBDIRS)
	for i in $(SUBDIRS); do \
		cd $$i; make clean ; make clean-deps; cd ..; done

examples:
	@cd Examples ; make ; cd ..

tools:
	@cd Tools ; make ; cd ..

show-examples:
	@cd Examples ; make show-examples ; cd ..

show-tools:
	@cd Tools ; make show-tools ; cd ..
