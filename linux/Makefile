#CC = gcc

#all: main
#main: libcalc.a main.c libshow.so
#    gcc -o main main.c -L. -lcalc  -lshow

#lib: libshare_data.a
#libshare_data.so
#libshare_data.a : share_data.o
        # ar crs libshare_data.a share_data.o
# share_data.o : share_data.c
        # $(CC) -c share_data.c -o share_data.o
#libshow.so: show.o
#        $(CC) -shared -o libshow.so show.o
#show.o: show.c
#        $(CC) -fPIC -Wall -c show.c -o show.o


#clean:
#    rm -rf *.o *.so *.a main


CC = gcc

lib: libshare_data.so

libshare_data.so : share_data.o share_data.h
	$(CC) -shared -o libshare_data.so share_data.o
share_data.o : share_data.c
	$(CC) -I ../include/common/ -c share_data.c -o share_data.o -lm