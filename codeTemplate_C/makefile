##############################
# STRUCTURE and INFORMATIONS #
##############################
#
# General structure of Makefile commands are: (target will builds if dependences change; target and dependences are files, action is a shell command)
# 	target: dependences
#		action
#
# first istruction is the only executed, in this case --all: bin/$(OUTPUT)-- others are executed if and only if dependences change
#
# to clean outputs folders digit: --make -f Makefile clean-- on the terminal
#
# .PHONY permit to use all and clean that are not files
#
# remember to write file dependences on new projects and to link in the executables
#

##########
# Macros #
##########
OUTPUT = main
CC = gcc
LD = gcc
INCDIR= -I.
CFLAGS = -g -O0 -Wall -c $(INCDIR)
LDFLAGS= -g -O0 -Wall
#CXXFLAGS = -g -O0 -Wall $(INCDIR)
ALLEG = `allegro-config --libs`
PTHREAD = -lpthread -lrt 
MATH = -lm


################
# Main Targets #
################
# all and clean are not real files
.PHONY: all clean

# how to make
all: bin/$(OUTPUT)

clean:
	rm -f bin/* build/*
	rmdir bin/ build/


################
# Object Files #
################
build/main.o: $(OUTPUT).c
	mkdir -p build/
	$(CC) $(CFLAGS) $< -o $@

build/%.o: %.c %.h
	mkdir -p build/
	$(CC) $(CFLAGS) $< -o $@


###############
# Executables #
###############
bin/main: build/$(OUTPUT).o build/global.o
	mkdir -p bin/
	$(LD) $(LDFLAGS) $^ -o $@ $(ALLEG) $(PTHREAD) $(MATH) 

