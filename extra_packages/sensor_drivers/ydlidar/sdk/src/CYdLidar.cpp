#include "CYdLidar.h"
#include "common.h"
#include <map>



using namespace std;
using namespace ydlidar;
using namespace impl;


/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar() : lidarPtr(nullptr)
{
    m_SerialPort        = "";
    m_SerialBaudrate    = 115200;
    m_Intensities       = false;
    m_FixedResolution   = false;
    m_Exposure          = false;
    m_Reversion         = false;
    m_AutoReconnect     = true;
    m_MaxAngle          = 180.f;
    m_MinAngle          = -180.f;
    m_MaxRange          = 16.0;
    m_MinRange          = 0.08;
    m_SampleRate        = 9;
    m_ScanFrequency     = 7;
    isScanning          = false;
    node_counts         = 720;
    each_angle          = 0.5;
    m_FrequencyOffset   = 0.4;
    m_isMultipleRate    = false;
    m_IgnoreArray.clear();
}

/*-------------------------------------------------------------
                    ~CYdLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar()
{
    disconnecting();
}

void CYdLidar::disconnecting()
{
    if (lidarPtr) {
        lidarPtr->disconnect();
        delete lidarPtr;
        lidarPtr = nullptr;
    }
}

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &outscan, bool &hardwareError){
	hardwareError			= false;

	// Bound?
    if (!checkHardware())
	{
        hardwareError = true;
        return false;
	}

    node_info nodes[3600];
    size_t   count = _countof(nodes);

    size_t all_nodes_counts = node_counts;

    //  wait Scan data:
    uint64_t tim_scan_start = getTime();
    result_t op_result =  lidarPtr->grabScanData(nodes, count);
    uint64_t tim_scan_end = getTime();

	// Fill in scan data:
    if (IS_OK(op_result))
	{
        op_result = lidarPtr->ascendScanData(nodes, count);
		//同步后的时间
        tim_scan_start = nodes[0].stamp;
        tim_scan_end   = nodes[0].stamp;

        if (IS_OK(op_result))
		{
            if(!m_FixedResolution){
                all_nodes_counts = count;
            } else {
                all_nodes_counts = node_counts;
            }
            each_angle = 360.0/all_nodes_counts;

            node_info *angle_compensate_nodes = new node_info[all_nodes_counts];
            memset(angle_compensate_nodes, 0, all_nodes_counts*sizeof(node_info));
            unsigned int i = 0;
            for( ; i < count; i++) {
                if (nodes[i].distance_q2 != 0) {
                    float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                    if(m_Reversion){
                       angle=angle+180;
                       if(angle>=360){ angle=angle-360;}
                        nodes[i].angle_q6_checkbit = ((uint16_t)(angle * 64.0f)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT;
                    }
                    int inter =(int)( angle / each_angle );
                    float angle_pre = angle - inter * each_angle;
                    float angle_next = (inter+1) * each_angle - angle;
                    if (angle_pre < angle_next) {
                        if(inter < all_nodes_counts)
                            angle_compensate_nodes[inter]=nodes[i];
                    } else {
                        if (inter < all_nodes_counts -1)
                            angle_compensate_nodes[inter+1]=nodes[i];
                    }
                }

                if(tim_scan_start > nodes[i].stamp) {
                    tim_scan_start = nodes[i].stamp;
                }
                if(tim_scan_end < nodes[i].stamp) {
                    tim_scan_end = nodes[i].stamp;
                }

             }

            LaserScan scan_msg;

            if (m_MaxAngle< m_MinAngle) {
                float temp = m_MinAngle;
                m_MinAngle = m_MaxAngle;
                m_MaxAngle = temp;
            }


            double scan_time = tim_scan_end - tim_scan_start;
            int counts = all_nodes_counts*((m_MaxAngle-m_MinAngle)/360.0f);
            int angle_start = 180+m_MinAngle;
            int node_start = all_nodes_counts*(angle_start/360.0f);

            scan_msg.ranges.resize(counts);
            scan_msg.intensities.resize(counts);
            float range = 0.0;
            float intensity = 0.0;
            int index = 0;


            for (size_t i = 0; i < all_nodes_counts; i++) {
                if(m_isMultipleRate) {
                    range = (float)angle_compensate_nodes[i].distance_q2/2000.f;
                }else {
                    range = (float)angle_compensate_nodes[i].distance_q2/4000.f;
                }
                intensity = (float)(angle_compensate_nodes[i].sync_quality >> LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

                if (i<all_nodes_counts/2) {
                    index = all_nodes_counts/2-1-i;
                } else {
                    index =all_nodes_counts-1-(i-all_nodes_counts/2);
                }

                if (m_IgnoreArray.size() != 0) {
                    float angle = (float)((angle_compensate_nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                    if (angle>180) {
                        angle=360-angle;
                    } else {
                        angle=-angle;
                    }

                    for (uint16_t j = 0; j < m_IgnoreArray.size();j = j+2) {
                        if ((m_IgnoreArray[j] < angle) && (angle <= m_IgnoreArray[j+1])) {
                           range = 0.0;
                           break;
                        }
                    }
                }

                if (range > m_MaxRange|| range < m_MinRange) {
                    range = 0.0;
                }

                int pos = index - node_start ;
                if (0<= pos && pos < counts) {
                    scan_msg.ranges[pos] =  range;
                    scan_msg.intensities[pos] = intensity;
                }
            }

            scan_msg.system_time_stamp = tim_scan_start;
            scan_msg.self_time_stamp = tim_scan_start;
            scan_msg.config.min_angle = DEG2RAD(m_MinAngle);
            scan_msg.config.max_angle = DEG2RAD(m_MaxAngle);
            scan_msg.config.ang_increment = (scan_msg.config.max_angle - scan_msg.config.min_angle) / (double)counts;
            scan_msg.config.time_increment = scan_time / (double)counts;
            scan_msg.config.scan_time = scan_time;
            scan_msg.config.min_range = m_MinRange;
            scan_msg.config.max_range = m_MaxRange;
            outscan = scan_msg;
            delete[] angle_compensate_nodes;
            return true;


		}

    } else {
        if (op_result==RESULT_FAIL) {
			// Error? Retry connection
			//this->disconnect();
		}
	}

	return false;

}


/*-------------------------------------------------------------
						turnOn
-------------------------------------------------------------*/
bool  CYdLidar::turnOn()
{
    bool ret = false;
    if (isScanning) {
        lidarPtr->startMotor();
        ret = true;
	}

	return ret;
}

/*-------------------------------------------------------------
						turnOff
-------------------------------------------------------------*/
bool  CYdLidar::turnOff()
{
    if (lidarPtr) {
        lidarPtr->stop();
        lidarPtr->stopMotor();
        isScanning = false;
	}
	return true;
}

/** Returns true if the device is connected & operative */
bool CYdLidar::getDeviceHealth() const {
    if (!lidarPtr) return false;

	result_t op_result;
    device_health healthinfo;

    op_result =lidarPtr->getHealth(healthinfo);
    if (IS_OK(op_result)) {
        if(healthinfo.status==0) {
            ydlidar::console.message("YDLidar running correctly ! The health status is good");
        } else {
            ydlidar::console.error("YDLidar running correctly ! The health status is bad");
        }
        if (healthinfo.status == 2) { 
            ydlidar::console.error("Error, Yd Lidar internal error detected. Please reboot the device to retry.");
            return false;
        } else {  
            return true;
        }

    } else { 
        ydlidar::console.error( "Error, cannot retrieve Yd Lidar health code: %x", op_result);
        return false;
    }

}

bool CYdLidar::getDeviceInfo(int &type) {

    if (!lidarPtr) return false;

	device_info devinfo;
    result_t ans = lidarPtr->getDeviceInfo(devinfo);
    if (!IS_OK(ans)) {   
        ydlidar::console.error("get DeviceInfo Error" );
		return false;
	}	 
	std::string model;
    sampling_rate _rate;
    int _samp_rate=4;
    int bad = 0;

    m_isMultipleRate = false;
    type = devinfo.model;
    switch (devinfo.model) {
        case YDlidarDriver::YDLIDAR_F4:
            model="F4";
            break;
        case YDlidarDriver::YDLIDAR_T1:
            model="T1";
            break;
        case YDlidarDriver::YDLIDAR_F2:
            model="F2";
            break;
        case YDlidarDriver::YDLIDAR_S4:
            model="S4";
            break;
        case YDlidarDriver::YDLIDAR_G4:
        {
            model="G4";
            ans = lidarPtr->getSamplingRate(_rate);
            if (IS_OK(ans)) {
                switch (m_SampleRate) {
                case 4:
                    _samp_rate=YDlidarDriver::YDLIDAR_RATE_4K;
                    break;
                case 8:
                    _samp_rate=YDlidarDriver::YDLIDAR_RATE_8K;
                    break;
                case 9:
                    _samp_rate=YDlidarDriver::YDLIDAR_RATE_9K;
                    break;
                default:
                    _samp_rate = _rate.rate;
                    break;
                }

                while (_samp_rate != _rate.rate) {
                    ans = lidarPtr->setSamplingRate(_rate);
                    if (!IS_OK(ans)) {
                        bad++;
                        if(bad>5){
                            break;
                        }
                    }
                }

                switch (_rate.rate) {
                    case YDlidarDriver::YDLIDAR_RATE_4K:
                        _samp_rate = 4;
                        break;
                    case YDlidarDriver::YDLIDAR_RATE_8K:
                        node_counts = 1440;
                        each_angle = 0.25;
                        _samp_rate=8;
                        break;
                    case YDlidarDriver::YDLIDAR_RATE_9K:
                        node_counts = 1440;
                        each_angle = 0.25;
                        _samp_rate=9;
                        break;
                    default:
                        break;
                }


            }

	    }
            break;
        case YDlidarDriver::YDLIDAR_X4:
            model="X4";
            break;
        case YDlidarDriver::YDLIDAR_G4PRO:
            model="G4Pro";
            break;
        case YDlidarDriver::YDLIDAR_F4PRO:
        {
            model="F4Pro";
            ans = lidarPtr->getSamplingRate(_rate);
            if (IS_OK(ans)) {
                switch (m_SampleRate) {
                case 4:
                    _samp_rate=0;
                    break;
                case 6:
                    _samp_rate=1;
                    break;
                default:
                    _samp_rate = _rate.rate;
                    break;
                }
                while (_samp_rate != _rate.rate) {
                    ans = lidarPtr->setSamplingRate(_rate);
                    if (!IS_OK(ans)) {
                        bad++;
                        if(bad>5){
                            break;
                        }
                    }
                }

                switch (_rate.rate) {
                    case 0:
                        _samp_rate = 4;
                        break;
                    case 1:
                        node_counts = 1440;
                        each_angle = 0.25;
                        _samp_rate=6;
                        break;
                }

            }

        }
            break;
        case  YDlidarDriver::YDLIDAR_G4C:
            model = "G4C";
            break;
        case  YDlidarDriver::YDLIDAR_G10:
            model = "G10";
             _samp_rate=10;
            break;
        case  YDlidarDriver::YDLIDAR_S4B:
            model = "S4B";
            break;
        case  YDlidarDriver::YDLIDAR_S2:
            model = "S2";
            break;
        case  YDlidarDriver::YDLIDAR_G25:
            model="G25";
            ans = lidarPtr->getSamplingRate(_rate);
            if (IS_OK(ans)) {
                switch (m_SampleRate) {
                case 8:
                    _samp_rate=YDlidarDriver::YDLIDAR_RATE_4K;
                    break;
                case 16:
                    _samp_rate=YDlidarDriver::YDLIDAR_RATE_8K;
                    break;
                case 18:
                    _samp_rate=YDlidarDriver::YDLIDAR_RATE_9K;
                    break;
                default:
                    _samp_rate = _rate.rate;
                    break;
                }

                while (_samp_rate != _rate.rate) {
                    ans = lidarPtr->setSamplingRate(_rate);
                    if (!IS_OK(ans)) {
                        bad++;
                        if(bad>5){
                            break;
                        }
                    }
                }

                switch (_rate.rate) {
                    case YDlidarDriver::YDLIDAR_RATE_4K:
                        _samp_rate = 8;
                        node_counts = 1440;
                        each_angle = 0.25;
                        break;
                    case YDlidarDriver::YDLIDAR_RATE_8K:
                        node_counts = 2400;
                        each_angle = 0.15;
                        _samp_rate=16;
                        break;
                    case YDlidarDriver::YDLIDAR_RATE_9K:
                        node_counts = 2600;
                        each_angle = 0.1;
                        _samp_rate=18;
                        break;
                    default:
                        break;
                }


            }
            m_isMultipleRate = true;

            break;
        default:
            model = "Unknown";
            break;
    }

    m_SampleRate = _samp_rate;
    lidarPtr->setMultipleRate(m_isMultipleRate);



    unsigned int maxv = (unsigned int)(devinfo.firmware_version>>8);
    unsigned int midv = (unsigned int)(devinfo.firmware_version&0xff)/10;
    unsigned int minv = (unsigned int)(devinfo.firmware_version&0xff)%10;
    ydlidar::console.show("[YDLIDAR] Connection established in [%s]:\n"
			   "Firmware version: %u.%u.%u\n"
			   "Hardware version: %u\n"
			   "Model: %s\n"
			   "Serial: ",
                m_SerialPort.c_str(),
			    maxv,
			    midv,
                minv,
			    (unsigned int)devinfo.hardware_version,
			    model.c_str());

		for (int i=0;i<16;i++)
            ydlidar::console.show("%01X",devinfo.serialnum[i]&0xff);
        ydlidar::console.show("\n");

        ydlidar::console.message("[YDLIDAR INFO] Current Sampling Rate : %dK" , _samp_rate);


        if (devinfo.model == YDlidarDriver::YDLIDAR_G4 ||
                devinfo.model ==YDlidarDriver::YDLIDAR_F4PRO ||
                devinfo.model == YDlidarDriver::YDLIDAR_G4C ||
                devinfo.model == YDlidarDriver::YDLIDAR_G10 ||
                devinfo.model == YDlidarDriver::YDLIDAR_G25) {
            checkScanFrequency();
        } else {
        }

		return true;
	

}

/*-------------------------------------------------------------
                        checkScanFrequency
-------------------------------------------------------------*/
bool CYdLidar::checkScanFrequency()
{
    float freq = 7.0f;
    scan_frequency _scan_frequency;
    int hz = 0;
    if (5 <= m_ScanFrequency && m_ScanFrequency <= 12) {
        result_t ans = lidarPtr->getScanFrequency(_scan_frequency) ;
        if (IS_OK(ans)) {
            freq = _scan_frequency.frequency/100.f;
            hz = m_ScanFrequency - freq;
            if (hz>0) {
                while (hz != 0) {
                    lidarPtr->setScanFrequencyAdd(_scan_frequency);
                    hz--;
                }
                freq = _scan_frequency.frequency/100.0f;
            } else {
                while (hz != 0) {
                    lidarPtr->setScanFrequencyDis(_scan_frequency);
                    hz++;
                }
                freq = _scan_frequency.frequency/100.0f;
            }
        }

        node_counts = m_SampleRate*1000/(m_ScanFrequency - m_FrequencyOffset);
        each_angle = 360.0/node_counts;
    }

    ydlidar::console.message("[YDLIDAR INFO] Current Scan Frequency : %fHz" , freq - m_FrequencyOffset);

    return true;

}

/*-------------------------------------------------------------
						checkCOMMs
-------------------------------------------------------------*/
bool  CYdLidar::checkCOMMs()
{
    if (!lidarPtr) {
        // create the driver instance
        lidarPtr = new YDlidarDriver();
        if (!lidarPtr) {
             ydlidar::console.error("Create Driver fail");
             return false;
        }
    }
    if (lidarPtr->isconnected()) {
        return true;
    }

	// Is it COMX, X>4? ->  "\\.\COMX"
    if (m_SerialPort.size()>=3) {
        if ( tolower( m_SerialPort[0]) =='c' && tolower( m_SerialPort[1]) =='o' && tolower( m_SerialPort[2]) =='m' ) {
			// Need to add "\\.\"?
            if (m_SerialPort.size()>4 || m_SerialPort[3]>'4')
                m_SerialPort = std::string("\\\\.\\") + m_SerialPort;
		}
	}

	// make connection...
    result_t op_result = lidarPtr->connect(m_SerialPort.c_str(), m_SerialBaudrate);
    if (!IS_OK(op_result)) {
        ydlidar::console.error("[CYdLidar] Error, cannot bind to the specified serial port %s",  m_SerialPort.c_str() );
		return false;
	}

	return true;
}

/*-------------------------------------------------------------
                        checkStatus
-------------------------------------------------------------*/
bool CYdLidar::checkStatus()
{

    if (!lidarPtr)
        return false;
    if (lidarPtr->isscanning())
        return true;

    std::map<int, bool> checkmodel;
    checkmodel.insert(std::map<int, bool>::value_type(115200, false));
    checkmodel.insert(std::map<int, bool>::value_type(128000, false));
    checkmodel.insert(std::map<int, bool>::value_type(153600, false));
    checkmodel.insert(std::map<int, bool>::value_type(230400, false));
    checkmodel.insert(std::map<int, bool>::value_type(512000, false));

    bool ret = false;
    int m_type = -1;

    while (!ret) {
        ret = getDeviceHealth();
        if(!ret) {
            delay(1000);
        }
        if(!getDeviceInfo(m_type)&&!ret) {
            checkmodel[m_SerialBaudrate] = true;
            map<int,bool>::iterator it;
            for (it=checkmodel.begin(); it!=checkmodel.end(); ++it) {
                if(it->second)
                    continue;
                lidarPtr->disconnect();
                delete lidarPtr;
                lidarPtr = nullptr;
                m_SerialBaudrate = it->first;

                if (!checkCOMMs()) {
                    return false;
                } else {
                    break;
                }
            }
            if(it == checkmodel.end()) {
                return false;
            }
        } else {
            ret = true;
            break;
        }
    }

    m_Intensities = false;
    if (m_type == YDlidarDriver::YDLIDAR_S4 || m_type == YDlidarDriver::YDLIDAR_S4B) {
        if (m_SerialBaudrate == 153600||m_type == YDlidarDriver::YDLIDAR_S4B)
            m_Intensities = true;

        if (m_Intensities) {
            scan_exposure exposure;
            int cnt = 0;
            while ((lidarPtr->setLowExposure(exposure) == RESULT_OK) && (cnt<3)) {
                if (exposure.exposure != m_Exposure) {
                    ydlidar::console.message("set EXPOSURE MODEL SUCCESS!!!");
                    break;
                }
                cnt++;
            }
            if (cnt >=4 ) {
                ydlidar::console.warning("set LOW EXPOSURE MODEL FALIED!!!");
            }
        }
    }

    lidarPtr->setIntensities(m_Intensities);

     // start scan...
    result_t s_result= lidarPtr->startScan();
    if (!IS_OK(s_result)) {
        s_result= lidarPtr->startScan();
        if (!IS_OK(s_result)) {
            ydlidar::console.error("[CYdLidar] Error starting scanning mode: %x", s_result);
            isScanning = false;
            return false;
        }
    }
    lidarPtr->setAutoReconnect(m_AutoReconnect);
    ydlidar::console.message("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
    isScanning = true;
    return true;

}

/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool CYdLidar::checkHardware()
{
    bool ret = true;
    if (!isScanning) {
        ret = checkCOMMs();
        if (ret&&(ret = checkStatus())) {
            if(ret) {
                ret = turnOn();
            }
        }
    }
    return ret;
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
bool CYdLidar::initialize()
{
	bool ret = true;
    if (!checkCOMMs()) {
         ydlidar::console.error("[CYdLidar::initialize] Error initializing YDLIDAR scanner.");
        return false;
	}
    if (!checkStatus()) {
         ydlidar::console.warning("[CYdLidar::initialize] Error initializing YDLIDAR scanner.because of failure in scan mode.");
    }
    if (!turnOn()) {
        ydlidar::console.warning("[CYdLidar::initialize] Error initializing YDLIDAR scanner. Because the motor falied to start.");
		
	}
    return ret;
	
}
