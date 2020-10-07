CC = g++ 
LIBS = -lm


#OBJS= allocate.o 
OBJS_VQS= vq.o

#vqs_regions: $(OBJS_VQS) $(OBJS)
	  #$(CC) -o $@ $(OBJS_VQS) $(OBJS) $(LIBS)
vq_r: $(OBJS_VQS)
	  $(CC) -o $@ $(OBJS_VQS) $(LIBS)

#$(OBJS):
	#$(CC) $(CFLAGS) -c $*.h 

$(OBJS_VQS):
	$(CC) $(CFLAGS) -c $*.cpp 

