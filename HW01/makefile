CC=gcc
CFLAGS=-lm
DEPS = intersections.h
OBJ = intersections.o \
      utility.o \
      main.o 

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

intersections: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

clean:
	rm -f ./*.o ./*~ intersections
