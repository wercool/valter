CXXFLAGS =	-O2 -g -Wall -fmessage-length=0

OBJS =		test.o

LIBS = -lglut -lGLU

TARGET =	test

$(TARGET):	$(OBJS)
	$(CXX) -o $(TARGET) $(OBJS) $(LIBS)

all:	$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
