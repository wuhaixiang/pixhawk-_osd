CC = gcc

lib: libshare_data.so

libshare_data.so : share_data.o share_data.h
	$(CC) -shared -o libshare_data.so share_data.o
share_data.o : share_data.c
	$(CC) -I ../c_library_v2/common/ -c share_data.c -o share_data.o -lm