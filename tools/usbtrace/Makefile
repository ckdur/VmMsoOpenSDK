libusbtrace.so: usbtrace.c
	$(CC) -O2 -fPIC -shared -o $@ $< -ldl -lpthread
clean:
	rm -f libusbtrace.so
