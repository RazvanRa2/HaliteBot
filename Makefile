all: build

build:
	mv ./Makefile ./AuxMakefile;
	cmake .; make;
	rm ./Makefile; rm -rf CMakeFiles; rm cmake_install.cmake; rm CMakeCache.txt;
	mv ./AuxMakefile ./Makefile

run:
	./MyBot

clean:
	rm ./MyBot
