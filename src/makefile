
CC = gcc
LFLAGS =  
INCLUDES = -I include/linux \
					 -I include/socketcan \
					 -I include/wiringPi \
					 -I include/wiringPi/wiringPi \
					 -I include/wiringPi/gpio \
					 -I ./ \
					 -I include/linux/can 


MAIN 		= candump
PARSE           = parse
CFLAGS 	= -Wall -g -o -Werror
LIBS 		= -lwiringPi -lm -pthread -lcrypt -lrt

SRC 		= candump.c lib.c kbhit.c ABE_ADCPi.c tmr.c

all:    $(MAIN) $(PARSE)
	  @echo  Compiled

$(PARSE): parse.c
	    $(CC) $(CFLAGS) -o $(PARSE) parse.c $(LFLAGS) $(LIBS)

$(MAIN): $(SRC)
	  $(CC) $(CFLAGS) $(INCLUDES) -o $(MAIN) $(SRC) $(LFLAGS) $(LIBS) 


clean:
	rm -rf $(MAIN)
	rm -rf *.o

