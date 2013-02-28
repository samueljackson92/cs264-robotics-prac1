g++ -o simulator `pkg-config --cflags playerc++` simple.cpp pcontroller.h pcontroller.cpp occupancygrid.h occupancygrid.cpp vectorutils.h vectorutils.cpp `pkg-config --libs playerc++`
