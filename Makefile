all:
	g++ -std=c++14 gen.cpp -o gen -lavcodec -lavformat -lavfilter -lavdevice -lavutil -lswresample -lswscale
