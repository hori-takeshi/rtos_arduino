#
# poll.mk
#

SERIALS := $(foreach i,$(shell mode /status | FIND "COM"),$(filter COM%,$(subst :,,$(i))))
BEFORE  := $(shell type serials.list)
FOUND   := $(filter-out $(BEFORE),$(SERIALS))
RESULT  := $(if $(FOUND),$(FOUND),NONE)

.PHONY: init poll clean poll-test

# initialize serials.list
init:
	@echo $(SERIALS) > serials.list

# get a new list, and compare it to previous one.
# if a new device is found, print to stdout.
# serials.list is updated for next polling.
poll:
	@echo $(RESULT)
	@echo $(SERIALS) >serials.list

# clean up serial.list
clean:
	-@rm serials.list

poll-test:
	@echo BEFORE: $(BEFORE)
	@echo NOW:    $(SERIALS)
	@echo $(RESULT)
	@echo $(SERIALS) >serials.list
