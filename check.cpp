#include <stdio.h>
#include <string.h>
#include "RssWrapper.h"
#include <string>
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"
//#include <dlfcn.h>

int main() {
//    auto handle = dlopen("librssw.so", RTLD_LAZY);
//    typedef std::string (*func)(void);
//    auto ssWorld = (func)dlsym( handle, "ssWorld");
    spdlog::info("start");
    spdlog::set_level(spdlog::level::debug);
    std::cout << "rssw versoin: " << version() << std::endl;
    FILE* f=fopen("input-check.dat","r");
    char buffer[1000];
    char* p;
    auto res=fread(buffer, 1000, 1, f);
    fclose(f);
    if( res != 0 ) {printf("file size too big!\n"); return(1);}
    printf("size: %ld\n", strlen(buffer));
    p=strtok(buffer, "\n");
    printf("%s\n", p);
    std::string sLane(p);
    p=strtok(NULL, "\n");
    printf("%s\n", p);
    std::string sEgo(p);
    p=strtok(NULL, "\n");
    printf("%s\n", p);
    std::string sOther(p);

    std::stringstream in(sLane);
    double x,y,length,width,heading,velocity,lid;
    in >> x >> y >> length >> width >> heading >> lid;
    std::cout << "lane: x=" << x << ", y=" << y << ", length=" << length << ", width=" << width << ", heading=" << heading << ", lid=" << lid << std::endl;
    Lane lane(x,y,length,width,heading,lid);
    std::stringstream in2(sEgo);
    in2 >> x >> y >> heading >> velocity;
    std::cout << "ego: x=" << x << ", y=" << y << ", heading=" << heading << ", velocity=" << velocity << std::endl;
    Vehicle ego(x,y,heading,velocity);
    std::stringstream in3(sOther);
    in3 >> x >> y >> heading >> velocity;
    std::cout << "other: x=" << x << ", y=" << y << ", heading=" << heading << ", velocity=" << velocity << std::endl;
    Vehicle other(x,y,heading,velocity);
    VControl control;
    Restriction restriction;
//    std::string sWorld = ssWorld();
    RssCheck(lane, ego, other, restriction);
    RssRestrict(restriction, control);
    std::cout << control.str();
    std::cout << ssWorld() << std::endl;
    std::cout << ssSituation() << std::endl;
    std::cout << ssState() << std::endl;
    std::cout << ssResponse() << std::endl;
    std::cout << ssRestriction() << std::endl;
    
//    auto lane=Lane(1,2,3,4,5);
//    printf("lane %f %f %f %f %f\n", lane.x, lane.y, lane.length, lane.width, lane.heading);
    return 1;
}
