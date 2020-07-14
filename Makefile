main: main.o plateio.o
	gcc -o main main.o plateio.o
main.o: main.c plateio.h
	gcc -c -g main.c
plateio.o: plateio.c plateio.h
	gcc -c -g plateio.c
