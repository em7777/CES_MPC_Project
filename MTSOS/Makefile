all: MTSOS
debug: MTSOS_debug
test: TEST

MTSOS: test_MTSOS.o barrier_front.o dynamics_front.o MTSOS.o csparse.o
	gcc -framework Accelerate MTSOS.o csparse.o barrier_front.o dynamics_front.o test_MTSOS.o -o test_MTSOS

TEST: MTSOS
	./test_MTSOS

MTSOS_debug: test_MTSOS_debug.o barrier_front_debug.o dynamics_front_debug.o MTSOS_debug.o csparse_debug.o
	gcc -g -O0 -framework Accelerate MTSOS_debug.o csparse_debug.o barrier_front_debug.o dynamics_front_debug.o test_MTSOS_debug.o -o test_MTSOS_debug

test_MTSOS.o: test_MTSOS.c
	gcc -c test_MTSOS.c

test_MTSOS_debug.o: test_MTSOS.c
	gcc -g -O0 -c test_MTSOS.c -o test_MTSOS_debug.o

barrier_front.o: barrier_front.c
	gcc -c barrier_front.c

barrier_front_debug.o: barrier_front.c
	gcc -g -O0 -c barrier_front.c -o barrier_front_debug.o

dynamics_front.o: dynamics_front.c
	gcc -c dynamics_front.c

dynamics_front_debug.o: dynamics_front.c
	gcc -g -O0 -c dynamics_front.c -o dynamics_front_debug.o

MTSOS.o: MTSOS.c
	gcc -c MTSOS.c

MTSOS_debug.o: MTSOS.c
	gcc -g -O0 -c MTSOS.c -o MTSOS_debug.o

csparse.o: csparse.c
	gcc -c csparse.c

csparse_debug.o: csparse.c
	gcc -g -O0 -c csparse.c -o csparse_debug.o

clean:
	rm -rf *.o test_MTSOS test_MTSOS_debug