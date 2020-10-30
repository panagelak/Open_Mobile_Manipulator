
#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <memory>

using namespace std;
using namespace ydlidar;
CYdLidar laser;


int main(int argc, char * argv[])
{
    printf("__   ______  _     ___ ____    _    ____  \n");
    printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
    printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
    printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
    printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
    printf("\n");
    fflush(stdout);


    std::string port;
    int baudrate;
    std::string intensity;
    ydlidar::init(argc, argv);

    std::map<std::string, std::string> ports =  ydlidar::YDlidarDriver::lidarPortList();
    std::map<std::string,std::string>::iterator it;
    if(ports.size()==1) {
        it = ports.begin();
        ydlidar::console.show("Lidar[%s] detected, whether to select current radar(yes/no)?:", it->first.c_str());
        std::string ok;
        std::cin>>ok;
        for (size_t i=0; i <ok.size(); i++) {
            ok[i] = tolower(ok[i]);
        }
        if(ok.find("yes") != std::string::npos ||atoi(ok.c_str()) == 1 ) {
            port = it->second;
        }else{
            ydlidar::console.message("Please enter the lidar serial port:");
            std::cin>>port;
        }
    }else {
        int id = 0;
        for(it = ports.begin(); it != ports.end(); it++) {
            ydlidar::console.show("%d. %s\n", id, it->first.c_str());
            id++;
        }

        if(ports.empty()) {
            ydlidar::console.show("Not Lidar was detected. Please enter the lidar serial port:");
            std::cin>>port;
        } else {
            while(ydlidar::ok()) {
                ydlidar::console.show("Please select the lidar port:");
                std::string number;
                std::cin>>number;
                if((size_t)atoi(number.c_str()) >= ports.size()) {
                    continue;
                }
                it = ports.begin();
                id = atoi(number.c_str());
                while (id) {
                    id--;
                    it++;
                }
                port = it->second;
                break;
            }
        }
    }
    std::vector<unsigned int> baudrateList;
    baudrateList.push_back(115200);
    baudrateList.push_back(128000);
    baudrateList.push_back(153600);
    baudrateList.push_back(230400);
    baudrateList.push_back(512000);


    for( unsigned int i = 0; i < baudrateList.size(); i ++) {
        ydlidar::console.show("%u. %u\n", i, baudrateList[i]);
    }
    while (ydlidar::ok()) {
        ydlidar::console.show("Please enter the lidar serial baud rate:");
        std::string index;
        std::cin>>index;
        if(atoi(index.c_str()) >= baudrateList.size()) {
            ydlidar::console.warning("Invalid serial number, Please re-select\n");
            continue;
        }
        baudrate = baudrateList[atoi(index.c_str())];
        break;

    }


    int intensities  = 0;
    ydlidar::console.show("0. false\n");
    ydlidar::console.show("1. true\n");
    while (ydlidar::ok()) {
        ydlidar::console.show("Please enter the lidar intensity:");
        std::cin>>intensity;
        if(atoi(intensity.c_str()) >= 2) {
            ydlidar::console.warning("Invalid serial number, Please re-select");
            continue;
        }
        intensities = atoi(intensity.c_str());
        break;
    }

    if(!ydlidar::ok()) {
        return 0;
    }

    laser.setSerialPort(port);
    laser.setSerialBaudrate(baudrate);
    laser.setIntensities(intensities);//intensity
    laser.setAutoReconnect(true);//hot plug
    laser.setMaxRange(16.0);
    laser.setMinRange(0.26);
    laser.setMaxAngle(180);
    laser.setMinAngle(-180);
    laser.setReversion(false);
    laser.setFixedResolution(false);
    laser.initialize();


    while(ydlidar::ok()){
		bool hardError;
		LaserScan scan;

		if(laser.doProcessSimple(scan, hardError )){
            for(int i =0; i < scan.ranges.size(); i++ ){
                float angle = scan.config.min_angle + i*scan.config.ang_increment;
                float dis = scan.ranges[i];

            }
            ydlidar::console.message("Scan received[%llu]: %u ranges",scan.self_time_stamp, (unsigned int)scan.ranges.size());
        } else {
            ydlidar::console.warning("Failed to get Lidar Data");
        }
		
	}

    laser.turnOff();
    laser.disconnecting();
    return 0;


}
