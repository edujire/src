CC = g++ 

LIBS = -lm

PRG_OBJS= get_hmm_vision.cpp 

OBJS= get_hmm_vision.o

get_hmm_vision: $(OBJS)
	  $(CC)  -o $@ $(OBJS) $(LIBS)

$(OBJS): $(PRG_OBJS)
	$(CC) $(CFLAGS) -c $*.cpp 

clean:
	rm *.o
	rm get_hmm_vision
	
	
	
	
	
