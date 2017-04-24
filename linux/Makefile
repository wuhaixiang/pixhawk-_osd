CC = gcc

all: mavlink_udp

mavlink_udp: mavlink_udp.c libshare_data.so
	$(CC) -o mavlink_udp mavlink_udp.c -L. -lshare_data -I ../c_library_v2/common/ -lm

lib: libshare_data.so
libshare_data.so: share_data.o share_data.h
	$(CC) -shared -o libshare_data.so share_data.o
share_data.o : share_data.c
	$(CC) -I ../c_library_v2/common/ -c share_data.c -o share_data.o -lm

clean:
	rm -rf *.o mavlink_udp *.so