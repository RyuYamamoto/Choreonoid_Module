CC		= g++
LDFLAGS = `pkg-config choreonoid-body-plugin openrtm-aist --libs --cflags` -lhrpsysBaseStub -shared -fPIC -lboost_system -lboost_filesystem -lrt -lpthread
CFLAGS	= -Wall
LIBS	= 
SRCS	= $(wildcard *.cpp)
TEXT	= $(wildcard *.txt)
TARGET	= $(SRCS:.cpp=.so)
DESTDIR = /usr
INSTALL = cp -p

.PHONY: clean

all: $(TARGET)

PDController.so : PDController.cpp
	$(CC) $(CFLAGS) -o $@ PDController.cpp $(LDFLAGS)

clean:
	rm -rf $(OBJS) $(TARGET)

install:
	mkdir -p $(DESTDIR)/lib/choreonoid-1.5/rtc
	$(INSTALL) $(TARGET) $(DESTDIR)/lib/choreonoid-1.5/rtc
	$(INSTALL) $(TARGET) $(DESTDIR)/lib/choreonoid-1.5/rtc
	$(INSTALL) $(TEXT) $(DESTDIR)/lib/choreonoid-1.5/rtc
