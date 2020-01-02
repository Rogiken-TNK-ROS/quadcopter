CONTROLLER=QuadcopterControllerWRS.so
SRC=QuadcopterControllerWRS.cpp
OBJ=$(SRC:%.cpp=%.o)
CHOREONOID_PLUGIN_SUBDIR = /home/taiki/wrs_ws/devel/lib/choreonoid-1.8

$(CONTROLLER): $(OBJ)
		g++ --shared -std=c++11 -o $(CONTROLLER) $(OBJ) `pkg-config --libs choreonoid-body`

%.o: %.cpp
		g++ -std=c++11 -fPIC `pkg-config --cflags choreonoid-body` -c $<

install: $(CONTROLLER)
		install -s $(CONTROLLER) `pkg-config --variable=simplecontrollerdir choreonoid-body`
clean:
		rm -f *.o *.so