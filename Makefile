PRG=gnu.exe
GCC=g++
GCCFLAGS=-O3 -Wall -Wextra -std=c++11

MSC=cl
MSCFLAGS=/EHa /W4 /Za /Zc:forScope /nologo /D_CRT_SECURE_NO_DEPRECATE /D"_SECURE_SCL 0" /O2i /GL

BCC=bcc32
BCCFLAGS=-w

VALGRIND_OPTIONS=-q --leak-check=full
DIFF_OPTIONS=-y --strip-trailing-cr --suppress-common-lines

OBJECTS0=
DRIVER0=driver.cpp

OSTYPE := $(shell uname)
ifeq ($(OSTYPE),Linux)
    CYGWIN=
    LM=
else
    LM=-lm
    CYGWIN=-Wl,--enable-auto-import
endif

gcc0:
	$(GCC) -o $(PRG) $(CYGWIN) $(DRIVER0) $(OBJECTS0) $(GCCFLAGS) $(LM)
0 1 4:
	echo "running test$@"
	@echo "should run in less than 200 ms"
	./$(PRG) $@ >studentout$@
	@echo "lines after the next are mismatches with master output -- see out$@"
	diff out$@ studentout$@ $(DIFF_OPTIONS)
9:
	echo "running test$@"
	@echo "should run in less than 2000 ms"
	./$(PRG) $@ >studentout$@
	@echo "lines after the next are mismatches with master output -- see out$@"
	diff out$@ studentout$@ $(DIFF_OPTIONS)
clean:
	rm -f *gcno *gcda *gcov *.exe *.o *.obj *.tds *.exe.manifest student* difference*
