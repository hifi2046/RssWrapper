zmain:
	g++ -I/usr/local/include -I/usr/include/python3.7m -L/usr/local/lib -L/usr/lib/x86_64-linux-gnu -fPIC -shared -o rssw.so RssWrapper.cpp -lad_rss -lad_map_access -lad_physics -lspdlog -lpython3.7m -lboost_python3
install:
	cp rssw.so ../InTraceControl
check:
	g++ -I/usr/local/include -I/usr/include/python3.7m -L/usr/local/lib -L/usr/lib/x86_64-linux-gnu -o check check.cpp RssWrapper.cpp -lad_rss -lad_map_access -lad_physics -lspdlog -lpython3.7m -lboost_python3