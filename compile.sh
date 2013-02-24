g++ -o simulator `pkg-config --cflags playerc++` simple.cpp pcontroller.h pcontroller.cpp occupancygrid.h occupancygrid.cpp `pkg-config --libs playerc++`
