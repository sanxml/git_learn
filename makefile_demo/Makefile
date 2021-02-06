objects = main.o input.o calcu.o
path = /run/media/sanxml/ubuntu/GitHub/QinYUN575/buildroot-licheepi/output/host/bin/arm-buildroot-linux-gnueabi-


main: $(objects)
	$(path)gcc -o main $(objects)

.PHONY : clean

%.o : %.c
	$(path)gcc -c $<

clean:
	rm *.o
	rm main
