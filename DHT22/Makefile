# Lets you run make in this sketch folder: every target is passed on to the Makefile in the
# repository root, building this sketch, e.g. `make` or `make upload PORT=/dev/ttyACM0`.
# Run `make help` to list all targets.

SKETCH := $(notdir $(CURDIR))
.DEFAULT_GOAL := compile

# FORCE makes the targets run even if a file or folder with the same name exists (e.g. test/)
%: FORCE
	@$(MAKE) --no-print-directory -C .. SKETCH=$(SKETCH) $@

FORCE: ;
Makefile: ;
