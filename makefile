main:
	g++ -I/usr/local/include -I/usr/include/python3.7m -L/usr/local/lib -L/usr/lib/x86_64-linux-gnu -fPIC -shared -o rssw.so RssWrapper.cpp -lad_rss -lad_map_access -lad_physics -lspdlog -lpython3.7m -lboost_python3
install:
	cp rssw.so ../InTraceControl
check: check.cpp RssWrapper.cpp
	g++ -I/usr/local/include -I/usr/include/python3.7m -L/usr/local/lib -L/usr/lib/x86_64-linux-gnu -o check check.cpp RssWrapper.cpp -lad_rss -lad_map_access -lad_physics -lspdlog -lpython3.7m -lboost_python3
debug:
	g++ -I/usr/local/include -I/usr/include/python3.7m -L/usr/local/lib -L/usr/lib/x86_64-linux-gnu -g -Wall -O3 -o check check.cpp RssWrapper.cpp -lad_rss -lad_map_access -lad_physics -lspdlog -lpython3.7m -lboost_python3
std:
	g++ -I/home/hifi/common/include -I/usr/include/python3.7m -L/home/hifi/common/lib -L/usr/lib/x86_64-linux-gnu -fPIC -shared -o rssw.so RssWrapper.cpp -lad_rss -lad_map_access -lad_physics -lspdlog -lpython3.7m -lboost_python3
check.std: check.cpp RssWrapper.cpp
	g++ -I/home/hifi/common/include -I/usr/include/python3.7m -L/home/hifi/common/lib -L/usr/lib/x86_64-linux-gnu -o checkstd check.cpp RssWrapper.cpp -lad_rss -lad_map_access -lad_physics -lspdlog -lpython3.7m -lboost_python3
debug.std:
	g++ -I/home/hifi/common/include -I/usr/include/python3.7m -L/home/hifi/common/lib -L/usr/lib/x86_64-linux-gnu -g -Wall -O3 -o checkstd check.cpp RssWrapper.cpp -lad_rss -lad_map_access -lad_physics -lspdlog -lpython3.7m -lboost_python3
