HEADERS = 
OBJECTS = easyBlack/src/memGPIO.o easyBlack/src/utils.o lathe.o

default: lathe pru0.bin

%.o: %.c $(HEADERS)
	gcc -c $< -o $@

%.o: %.cpp $(HEADERS)
	g++ -std=c++0x -O2 -c $< -o $@

lathe: $(OBJECTS)
	gcc $(OBJECTS) -o $@ -lprussdrv -lm -lstdc++

pru0.bin: pru0.p
	pasm -b $<

clean:
	-rm -f $(OBJECTS)
	-rm -f lathe
	-rm -f pru0.bin
