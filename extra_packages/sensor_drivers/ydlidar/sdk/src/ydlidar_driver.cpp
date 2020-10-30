/*
*  YDLIDAR SYSTEM
*  YDLIDAR DRIVER
*
*  Copyright 2015 - 2018 EAI TEAM
*  http://www.eaibot.com
* 
*/
#include "common.h"
#include "ydlidar_driver.h"
#include <math.h>
using namespace impl;

namespace ydlidar{

	YDlidarDriver::YDlidarDriver():
	_serial(0) {
		isConnected = false;
		isScanning = false;
        //串口配置参数
		m_intensities = false;
        isAutoReconnect = true;
        isAutoconnting = false;
		isMultipleRate = false;
		scan_node_count = 0;

        m_baudrate = 115200;
		isSupportMotorCtrl=true;
        m_sampling_rate=-1;
		model = -1;
        scan_frequence = 0;

        //解析参数
        PackageSampleBytes = 2;
        package_Sample_Index = 0;
        IntervalSampleAngle = 0.0;
        IntervalSampleAngle_LastPackage = 0.0;
        FirstSampleAngle = 0;
        LastSampleAngle = 0;
        CheckSun = 0;
        CheckSunCal = 0;
        SampleNumlAndCTCal = 0;
        LastSampleAngleCal = 0;
        CheckSunResult = true;
        Valu8Tou16 = 0;
    
	}

	YDlidarDriver::~YDlidarDriver(){
		{
			isScanning = false;
		}

        isAutoReconnect = false;
		_thread.join();

        ScopedLocker lk(_serial_lock);
		if(_serial){
			if(_serial->isOpen()){
				_serial->close();
			}
		}
		if(_serial){
			delete _serial;
			_serial = NULL;
		}

	}

    std::map<std::string, std::string>  YDlidarDriver::lidarPortList() {
        std::vector<PortInfo> lst = list_ports();
        std::map<std::string, std::string> ports;
        for(std::vector<PortInfo>::iterator it = lst.begin(); it != lst.end(); it++) {
            std::string port = "ydlidar" + (*it).device_id;
            ports[port] = (*it).port;
        }
        return ports;
    }

	result_t YDlidarDriver::connect(const char * port_path, uint32_t baudrate) {
        m_baudrate = baudrate;
        serial_port = string(port_path);
        ScopedLocker lk(_serial_lock);
        if(!_serial){
            _serial = new serial::Serial(port_path, m_baudrate, serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT));
        }

        {
            ScopedLocker l(_lock);
            if(!_serial->open()){
                return RESULT_FAIL;
            }
            isConnected = true;
        }


		{
			ScopedLocker l(_lock);
			sendCommand(LIDAR_CMD_FORCE_STOP);
			sendCommand(LIDAR_CMD_STOP);
		}

		clearDTR();

		return RESULT_OK;
	}


	void YDlidarDriver::setDTR() {
		if (!isConnected){
			return ;
		}
		if(_serial){
			_serial->setDTR(1);
			_serial->flush();
		}

	}

	void YDlidarDriver::clearDTR() {
		if (!isConnected){
			return ;
		}
		if(_serial){
			_serial->setDTR(0);
			_serial->flush();
		}
	}

	result_t YDlidarDriver::startMotor() {
		ScopedLocker l(_lock);
		if(isSupportMotorCtrl){
			setDTR();
			delay(500);
		}else{
			clearDTR();
			delay(500);
		}
		return RESULT_OK;
	}

	result_t YDlidarDriver::stopMotor() {
		ScopedLocker l(_lock);
		if(isSupportMotorCtrl){
			clearDTR();
			delay(500);
		}else{
			setDTR();
			delay(500);
		}
		return RESULT_OK;
	}

	void YDlidarDriver::disconnect() {
        isAutoReconnect = false;
		if (!isConnected){
			return ;
		}
		stop();

        ScopedLocker l(_serial_lock);
		if(_serial){
			if(_serial->isOpen()){
				_serial->close();
			}
		}
		isConnected = false;
	}


	void YDlidarDriver::disableDataGrabbing() {
		{
			if(isScanning) {
				isScanning = false;
				_dataEvent.set();
			}
		}
		_thread.join();
	}

    bool YDlidarDriver::isscanning() const
	{
		return isScanning;
	}
    bool YDlidarDriver::isconnected() const
    {
        return isConnected;
    }

	result_t YDlidarDriver::sendCommand(uint8_t cmd, const void * payload, size_t payloadsize) {
		uint8_t pkt_header[10];
		cmd_packet * header = reinterpret_cast<cmd_packet * >(pkt_header);
		uint8_t checksum = 0;

		if (!isConnected) {
			return RESULT_FAIL;
		}
		if (payloadsize && payload) {
			cmd |= LIDAR_CMDFLAG_HAS_PAYLOAD;
		}

		header->syncByte = LIDAR_CMD_SYNC_BYTE;
		header->cmd_flag = cmd;
		sendData(pkt_header, 2) ;

		if ((cmd & LIDAR_CMDFLAG_HAS_PAYLOAD)&&payloadsize && payload) {
			checksum ^= LIDAR_CMD_SYNC_BYTE;
			checksum ^= cmd;
			checksum ^= (payloadsize & 0xFF);

			uint8_t sizebyte = (uint8_t)(payloadsize);
			sendData(&sizebyte, 1);

			for (size_t pos = 0; pos < payloadsize; ++pos) {
				checksum ^= ((uint8_t *)payload)[pos];
			}

			sendData((const uint8_t *)payload, sizebyte);
			sendData(&checksum, 1);
		}
		return RESULT_OK;
	}

	result_t YDlidarDriver::sendData(const uint8_t * data, size_t size) {
		if (!isConnected) {
			return RESULT_FAIL;
		}

		if (data == NULL || size ==0) {
			return RESULT_FAIL;
		}
		size_t r;
        while (size) {
            r = _serial->write(data, size);
            if (r < 1) {
                return RESULT_FAIL;
            }
            size -= r;
            data += r;
        }
        return RESULT_OK;

	}

	result_t YDlidarDriver::getData(uint8_t * data, size_t size) {
		if (!isConnected) {
			return RESULT_FAIL;
		}
		size_t r;
        while (size) {
            r = _serial->read(data, size);
            if (r < 1)
                return RESULT_FAIL;
            size -= r;
            data += r;

        }
        return RESULT_OK;

	}

	result_t YDlidarDriver::waitResponseHeader(lidar_ans_header * header, uint32_t timeout) {
		int  recvPos = 0;
		uint32_t startTs = getms();
		uint8_t  recvBuffer[sizeof(lidar_ans_header)];
		uint8_t  *headerBuffer = reinterpret_cast<uint8_t *>(header);
		uint32_t waitTime;

		while ((waitTime=getms() - startTs) <= timeout) {
			size_t remainSize = sizeof(lidar_ans_header) - recvPos;
			size_t recvSize;

			result_t ans = waitForData(remainSize, timeout - waitTime, &recvSize);
            if (!IS_OK(ans)){
				return ans;
			}

			if(recvSize > remainSize) recvSize = remainSize;

			ans = getData(recvBuffer, recvSize);
            if (IS_FAIL(ans)){
                return ans;
			}

			for (size_t pos = 0; pos < recvSize; ++pos) {
				uint8_t currentByte = recvBuffer[pos];
				switch (recvPos) {
				case 0:
					if (currentByte != LIDAR_ANS_SYNC_BYTE1) {
						continue;
					}
					break;
				case 1:
					if (currentByte != LIDAR_ANS_SYNC_BYTE2) {
						recvPos = 0;
						continue;
					}
					break;
				}
				headerBuffer[recvPos++] = currentByte;

				if (recvPos == sizeof(lidar_ans_header)) {
					return RESULT_OK;
				}
			}
		}
		return RESULT_FAIL;
	}

	result_t YDlidarDriver::waitForData(size_t data_count, uint32_t timeout, size_t * returned_size) {
		size_t length = 0;
		if (returned_size==NULL) {
			returned_size=(size_t *)&length;
		}
		return (result_t)_serial->waitfordata(data_count, timeout, returned_size);
	}

    bool YDlidarDriver::autoReconnectLidar() {
        result_t ans;
        {
            ScopedLocker l(_serial_lock);
            if(_serial){
                if(_serial->isOpen()){
                    _serial->close();

                }
                delete _serial;
                _serial = NULL;
                isConnected = false;
            }
        }
        while(isAutoReconnect&&connect(serial_port.c_str(), m_baudrate) != RESULT_OK){
            delay(200);
        }
        if(!isAutoReconnect) {
            isAutoconnting = false;
            isScanning = false;
            return false;
        }
        if(isconnected()) {
            {
                ScopedLocker lk(_serial_lock);
                ans = startAutoScan();
                if(!IS_OK(ans)) {
                    ans = startAutoScan();
                }
            }
            if(IS_OK(ans)){
                isAutoconnting = false;
                return true;;
            }

        }
        return false;
    }

	int YDlidarDriver::cacheScanData() {
		node_info      local_buf[128];
		size_t         count = 128;
		node_info      local_scan[MAX_SCAN_NODES];
		size_t         scan_count = 0;
		result_t            ans;
		memset(local_scan, 0, sizeof(local_scan));
		waitScanData(local_buf, count);

        int timeout_count = 0;
		while(isScanning) {
			if ((ans=waitScanData(local_buf, count)) != RESULT_OK) {
                if (!IS_TIMEOUT(ans) || timeout_count > DEFAULT_TIMEOUT_COUNT ) {
                    if(!isAutoReconnect) {//不重新连接, 退出线程
                        fprintf(stderr, "exit scanning thread!!\n");
                        fflush(stderr);
                        {
                            isScanning = false;
                        }
                        return RESULT_FAIL;
                    } else {//做异常处理, 重新连接
                        isAutoconnting = true;
						while(isAutoReconnect&&isAutoconnting) {
                            ans = autoReconnectLidar();
                            if(IS_OK(ans)) {
                                timeout_count =0;
                                isAutoconnting = false;
                                continue;
                            }
						}
                    }

                } else {
                     timeout_count++;
                }
			}else {
				timeout_count = 0;
			}
			for (size_t pos = 0; pos < count; ++pos) {
				if (local_buf[pos].sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT) {
					if ((local_scan[0].sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT)) {
						_lock.lock();//timeout lock, wait resource copy 
						memcpy(scan_node_buf, local_scan, scan_count*sizeof(node_info));
						scan_node_count = scan_count;
						_dataEvent.set();
						_lock.unlock();
					}
					scan_count = 0;
				}
				local_scan[scan_count++] = local_buf[pos];
				if (scan_count == _countof(local_scan)){
					scan_count-=1;
				}
			}
		}
        isScanning = false;

		return RESULT_OK;
	}

	result_t YDlidarDriver::waitPackage(node_info * node, uint32_t timeout) {
		int recvPos = 0;
		uint32_t startTs = getms();
		uint32_t size = (m_intensities)?sizeof(node_package):sizeof(node_packages);
		uint8_t* recvBuffer = new uint8_t[size];

		uint32_t waitTime = 0;
		uint8_t *packageBuffer = (m_intensities)?(uint8_t*)&package.package_Head:(uint8_t*)&packages.package_Head;
		uint8_t  package_Sample_Num = 0;
		int32_t AngleCorrectForDistance = 0;
		int  package_recvPos = 0;
        uint8_t package_type = 0;
		bool eol             = false;

		if(package_Sample_Index == 0) {
			recvPos = 0;
			while ((waitTime=getms() - startTs) <= timeout) {
				size_t remainSize = PackagePaidBytes - recvPos;
				size_t recvSize;
				result_t ans = waitForData(remainSize, timeout-waitTime, &recvSize);
                if (!IS_OK(ans)){
					delete[] recvBuffer;
					return ans;
				}

				if (recvSize > remainSize){
					recvSize = remainSize;
				}

				getData(recvBuffer, recvSize);

				for (size_t pos = 0; pos < recvSize; ++pos) {
					uint8_t currentByte = recvBuffer[pos];
					switch (recvPos) {
					case 0:
						if(currentByte==(PH&0xFF)){
                            if (eol){
                                eol = false;
                            }
						}else{
                            if (!eol){
                                eol = true;
                            }
							continue;
						}
						break;
					case 1:
						CheckSunCal = PH;
						if(currentByte==(PH>>8)){

						}else{
							recvPos = 0;
							continue;
						}
						break;
					case 2:
						SampleNumlAndCTCal = currentByte;
                        package_type = currentByte&0x01;
                        if ((package_type == CT_Normal) || (package_type == CT_RingStart)){
                            if(package_type == CT_RingStart){
                                scan_frequence = (currentByte&0xFE)>>1;
                            }
						} else {
							recvPos = 0;
							continue;
						}
						break;
					case 3:
						SampleNumlAndCTCal += (currentByte*0x100);
						package_Sample_Num = currentByte;
						break;
					case 4:
						if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
							FirstSampleAngle = currentByte;
						} else {
							recvPos = 0;
							continue;
						}
						break;
					case 5:
						FirstSampleAngle += currentByte*0x100;
						CheckSunCal ^= FirstSampleAngle;
						FirstSampleAngle = FirstSampleAngle>>1;
						break;
					case 6:
						if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
							LastSampleAngle = currentByte;
						} else {
							recvPos = 0;
							continue;
						}
						break;
					case 7:
						LastSampleAngle = currentByte*0x100 + LastSampleAngle;
						LastSampleAngleCal = LastSampleAngle;
						LastSampleAngle = LastSampleAngle>>1;
						if(package_Sample_Num == 1){
							IntervalSampleAngle = 0;
						}else{
							if(LastSampleAngle < FirstSampleAngle){
                                if((FirstSampleAngle >= 180*64) && (LastSampleAngle <= 180*64)){//实际雷达跨度不超过60度
									IntervalSampleAngle = (float)((360*64 + LastSampleAngle - FirstSampleAngle)/((package_Sample_Num-1)*1.0));
									IntervalSampleAngle_LastPackage = IntervalSampleAngle;
                                } else{//这里不应该发生
                                    if( FirstSampleAngle > 360) {///< 负数
                                        IntervalSampleAngle = ((float)(LastSampleAngle - ((int16_t)FirstSampleAngle)))/(package_Sample_Num-1);
                                    } else {//起始角大于结束角
                                        uint16_t temp = FirstSampleAngle;
                                        FirstSampleAngle = LastSampleAngle;
                                        LastSampleAngle = temp;
                                        IntervalSampleAngle = (float)((LastSampleAngle -FirstSampleAngle)/((package_Sample_Num-1)*1.0));
                                    }
                                     IntervalSampleAngle_LastPackage = IntervalSampleAngle;
                                    //IntervalSampleAngle = IntervalSampleAngle_LastPackage;
								}
							} else{
								IntervalSampleAngle = (float)((LastSampleAngle -FirstSampleAngle)/((package_Sample_Num-1)*1.0));
								IntervalSampleAngle_LastPackage = IntervalSampleAngle;
							}
						}
						break;
					case 8:
						CheckSun = currentByte;	
						break;
					case 9:
						CheckSun += (currentByte*0x100);
						break;
					}
					packageBuffer[recvPos++] = currentByte;
				}

				if (recvPos  == PackagePaidBytes ){
					package_recvPos = recvPos;
					break;
				}
			}

			if(PackagePaidBytes == recvPos){
				startTs = getms();
				recvPos = 0;
				while ((waitTime=getms() - startTs) <= timeout) {
					size_t remainSize = package_Sample_Num*PackageSampleBytes - recvPos;
					size_t recvSize;
					result_t ans =waitForData(remainSize, timeout-waitTime, &recvSize);
                    if (!IS_OK(ans)){
						delete[] recvBuffer;
						return ans;
					}

					if (recvSize > remainSize){
						recvSize = remainSize;
					}

					getData(recvBuffer, recvSize);

					for (size_t pos = 0; pos < recvSize; ++pos) {
						if(m_intensities){
							if(recvPos%3 == 2){
								Valu8Tou16 += recvBuffer[pos]*0x100;
								CheckSunCal ^= Valu8Tou16;
							}else if(recvPos%3 == 1){
								Valu8Tou16 = recvBuffer[pos];
							}else{
								CheckSunCal ^= recvBuffer[pos]; 
							}
						}else{
							if(recvPos%2 == 1){
								Valu8Tou16 += recvBuffer[pos]*0x100;
								CheckSunCal ^= Valu8Tou16;
							}else{
								Valu8Tou16 = recvBuffer[pos];	
							}
						}				

						packageBuffer[package_recvPos+recvPos] = recvBuffer[pos];
						recvPos++;
					}

					if(package_Sample_Num*PackageSampleBytes == recvPos){
						package_recvPos += recvPos;
						break;
					}
				}
				if(package_Sample_Num*PackageSampleBytes != recvPos){
					delete[] recvBuffer;
					return RESULT_FAIL;
				}
			} else {
				delete[] recvBuffer;
				return RESULT_FAIL;
			}
			CheckSunCal ^= SampleNumlAndCTCal;
			CheckSunCal ^= LastSampleAngleCal;

			if(CheckSunCal != CheckSun){	
				CheckSunResult = false;
			}else{
				CheckSunResult = true;
			}

		}
		uint8_t package_CT;
		if(m_intensities){
			package_CT = package.package_CT;
		}else{
			package_CT = packages.package_CT;    
		}

		if(package_CT == CT_Normal){
            (*node).sync_flag =  Node_NotSync;
		} else{
            (*node).sync_flag =  Node_Sync;
		}
        (*node).sync_quality = Node_Default_Quality;

		if(CheckSunResult){
			if(m_intensities){
				if(isMultipleRate) {
                    (*node).sync_quality = (((package.packageSample[package_Sample_Index].PakageSampleDistance&0x01)<<LIDAR_RESP_MEASUREMENT_SYNC_QUALITY_SHIFT)| (package.packageSample[package_Sample_Index].PakageSampleQuality));
                    (*node).distance_q2 = package.packageSample[package_Sample_Index].PakageSampleDistance&0xfffe;
				} else {
                    (*node).sync_quality = (((package.packageSample[package_Sample_Index].PakageSampleDistance&0x03)<<LIDAR_RESP_MEASUREMENT_SYNC_QUALITY_SHIFT)| (package.packageSample[package_Sample_Index].PakageSampleQuality));
                    (*node).distance_q2 = package.packageSample[package_Sample_Index].PakageSampleDistance&0xfffc;
				}
			}else{			
				(*node).distance_q2 = packages.packageSampleDistance[package_Sample_Index];
			}	  

		
			if((*node).distance_q2 != 0){
				if(isMultipleRate) {
					AngleCorrectForDistance = (int32_t)(((atan(((21.8*(155.3 - ((*node).distance_q2/2.0)) )/155.3)/((*node).distance_q2/2.0)))*180.0/3.1415) * 64.0);
				}else {
					AngleCorrectForDistance = (int32_t)(((atan(((21.8*(155.3 - ((*node).distance_q2/4.0)) )/155.3)/((*node).distance_q2/4.0)))*180.0/3.1415) * 64.0);
				}
			}else{
				AngleCorrectForDistance = 0;		
			}

			if((FirstSampleAngle + IntervalSampleAngle*package_Sample_Index + AngleCorrectForDistance) < 0){
				(*node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + IntervalSampleAngle*package_Sample_Index + AngleCorrectForDistance + 360*64))<<1) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
			}else{
				if((FirstSampleAngle + IntervalSampleAngle*package_Sample_Index + AngleCorrectForDistance) > 360*64){
					(*node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + IntervalSampleAngle*package_Sample_Index + AngleCorrectForDistance - 360*64))<<1) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
				}else{
					(*node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + IntervalSampleAngle*package_Sample_Index + AngleCorrectForDistance))<<1) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
				} 
			}
		}else{
			(*node).sync_flag = Node_NotSync;
            (*node).sync_quality = Node_Default_Quality;
			(*node).angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
			(*node).distance_q2 = 0;
            scan_frequence = 0;
		}


		uint8_t nowPackageNum;
		if(m_intensities){
			nowPackageNum = package.nowPackageNum;
		}else{
			nowPackageNum = packages.nowPackageNum;
		}

		if((*node).sync_flag&LIDAR_RESP_MEASUREMENT_SYNCBIT){
            m_last_ns = m_ns;
			m_ns = getTime() - (nowPackageNum*3 +10)*trans_delay - (nowPackageNum -1)*m_pointTime;
            if(m_ns < m_last_ns) {
                m_ns = m_last_ns;
            }
		}

		(*node).stamp = m_ns + package_Sample_Index*m_pointTime;
        (*node).scan_frequence = scan_frequence;
		package_Sample_Index++;

		if(package_Sample_Index >= nowPackageNum){
			package_Sample_Index = 0;
             m_ns= (*node).stamp + m_pointTime;
		}
		delete[] recvBuffer;
		return RESULT_OK;
	}

	result_t YDlidarDriver::waitScanData(node_info * nodebuffer, size_t & count, uint32_t timeout) {
		if (!isConnected) {
			count = 0;
			return RESULT_FAIL;
		}

		size_t     recvNodeCount =  0;
		uint32_t   startTs = getms();
        uint32_t   waitTime = 0;
		result_t ans;

		while ((waitTime = getms() - startTs) <= timeout && recvNodeCount < count) {
			node_info node;
			if ((ans = this->waitPackage(&node, timeout - waitTime)) != RESULT_OK) {
				return ans;
			}
			nodebuffer[recvNodeCount++] = node;

			if (recvNodeCount == count) {
				return RESULT_OK;
			}
		}
		count = recvNodeCount;
		return RESULT_FAIL;
	}


	result_t YDlidarDriver::grabScanData(node_info * nodebuffer, size_t & count, uint32_t timeout) {
		switch (_dataEvent.wait(timeout)) {
		case Event::EVENT_TIMEOUT:
			count = 0;
			return RESULT_TIMEOUT;
		case Event::EVENT_OK:
			{
				if(scan_node_count == 0) {
					return RESULT_FAIL;
				}
								
				ScopedLocker l(_lock);
				size_t size_to_copy = min(count, scan_node_count);
				memcpy(nodebuffer, scan_node_buf, size_to_copy*sizeof(node_info));
				count = size_to_copy;
				scan_node_count = 0;
			}
			return RESULT_OK;
		default:
			count = 0;
			return RESULT_FAIL;
		}

	}

	result_t YDlidarDriver::ascendScanData(node_info * nodebuffer, size_t count) {
		float inc_origin_angle = (float)360.0/count;
		int i = 0;

		for (i = 0; i < (int)count; i++) {
			if(nodebuffer[i].distance_q2 == 0) {
				continue;
			} else {
				while(i != 0) {
					i--;
					float expect_angle = (nodebuffer[i+1].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f - inc_origin_angle;
					if (expect_angle < 0.0f) expect_angle = 0.0f;
					uint16_t checkbit = nodebuffer[i].angle_q6_checkbit & LIDAR_RESP_MEASUREMENT_CHECKBIT;
					nodebuffer[i].angle_q6_checkbit = (((uint16_t)(expect_angle * 64.0f)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
				}
				break;
			}
		}

		if (i == (int)count){
			return RESULT_FAIL;
		}

		for (i = (int)count - 1; i >= 0; i--) {
			if(nodebuffer[i].distance_q2 == 0) {
				continue;
			} else {
				while(i != ((int)count - 1)) {
					i++;
					float expect_angle = (nodebuffer[i-1].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f + inc_origin_angle;
					if (expect_angle > 360.0f) expect_angle -= 360.0f;
					uint16_t checkbit = nodebuffer[i].angle_q6_checkbit & LIDAR_RESP_MEASUREMENT_CHECKBIT;
					nodebuffer[i].angle_q6_checkbit = (((uint16_t)(expect_angle * 64.0f)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
				}
				break;
			}
		}

		float frontAngle = (nodebuffer[0].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
		for (i = 1; i < (int)count; i++) {
			if(nodebuffer[i].distance_q2 == 0) {
				float expect_angle =  frontAngle + i * inc_origin_angle;
				if (expect_angle > 360.0f) expect_angle -= 360.0f;
				uint16_t checkbit = nodebuffer[i].angle_q6_checkbit & LIDAR_RESP_MEASUREMENT_CHECKBIT;
				nodebuffer[i].angle_q6_checkbit = (((uint16_t)(expect_angle * 64.0f)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
			}
		}

		size_t zero_pos = 0;
		float pre_degree = (nodebuffer[0].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;

		for (i = 1; i < (int)count ; ++i) {
			float degree = (nodebuffer[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
			if (zero_pos == 0 && (pre_degree - degree > 180)) {
				zero_pos = i;
				break;
			}
			pre_degree = degree;
		}

		node_info *tmpbuffer = new node_info[count];
		for (i = (int)zero_pos; i < (int)count; i++) {
			tmpbuffer[i-zero_pos] = nodebuffer[i];
		}
		for (i = 0; i < (int)zero_pos; i++) {
			tmpbuffer[i+(int)count-zero_pos] = nodebuffer[i];
		}

		memcpy(nodebuffer, tmpbuffer, count*sizeof(node_info));
		delete[] tmpbuffer;

		return RESULT_OK;
	}

	/************************************************************************/
	/* get health state of lidar                                            */
	/************************************************************************/
	result_t YDlidarDriver::getHealth(device_health & health, uint32_t timeout) {
		result_t ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}

		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_GET_DEVICE_HEALTH)) != RESULT_OK) {
				return ans;
			}
			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVHEALTH) {
				return RESULT_FAIL;
			}

			if (response_header.size < sizeof(device_health)) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}

			getData(reinterpret_cast<uint8_t *>(&health), sizeof(health));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/* get device info of lidar                                             */
	/************************************************************************/
	result_t YDlidarDriver::getDeviceInfo(device_info & info, uint32_t timeout) {
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}

		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_GET_DEVICE_INFO)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size < sizeof(lidar_ans_header)) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&info), sizeof(info));
			model = info.model;
		}

		return RESULT_OK;
	}

	/************************************************************************/
	/* the set to signal quality                                            */
	/************************************************************************/
    void YDlidarDriver::setIntensities(const bool& isintensities){
		m_intensities = isintensities;
		if(m_intensities){
			PackageSampleBytes = 3;
		}else{
			PackageSampleBytes = 2;
		}
	}

    /**
        * @brief 设置雷达异常自动重新连接 \n
        * @param[in] enable    是否开启自动重连:
        *     true	开启
        *	  false 关闭
        */
    void YDlidarDriver::setAutoReconnect(const bool& enable) {  
        isAutoReconnect = enable;
    }

	/**
         * @brief 设置雷达采样倍频 \n
         * @param[in] enable    是否开启采样倍频:
         *     true	开启
         *	  false 关闭
         */
    void YDlidarDriver::setMultipleRate(const bool& enable) {
		isMultipleRate = enable;
	}

	bool YDlidarDriver::getMultipleRate() const {
		return isMultipleRate;
	}

	/************************************************************************/
	/* check one byte transform time                                        */
	/************************************************************************/
	void YDlidarDriver::checkTransTime() {
		{
            //calc stamp
            m_pointTime = 1e9/4000;
            trans_delay = 0;
            switch(model){
            case YDLIDAR_F4://f4
                trans_delay = _serial->getByteTime();
                break;
            case YDLIDAR_G4://g4
                if(m_sampling_rate == -1){
                    sampling_rate _rate;
                    getSamplingRate(_rate);
                    m_sampling_rate = _rate.rate;
                }
                switch(m_sampling_rate){
                case 1:
                    m_pointTime = 1e9/8000;
                    break;
                case 2:
                    m_pointTime = 1e9/9000;
                    break;
                }
                trans_delay = _serial->getByteTime();
                break;
            case YDLIDAR_X4://x4
                m_pointTime = 1e9/5000;
                break;
            case YDLIDAR_G4PRO://x4
                m_pointTime = 1e9/4000;
                break;
            case YDLIDAR_F4PRO://f4pro
                if(m_sampling_rate == -1){
                    sampling_rate _rate;
                    getSamplingRate(_rate);
                    m_sampling_rate = _rate.rate;
                }
                if(m_sampling_rate ==1){
                    m_pointTime = 1e9/6000;
                }
                trans_delay = _serial->getByteTime();
                break;
            case YDLIDAR_G4C://g4c
                trans_delay = _serial->getByteTime();
                break;
            case YDLIDAR_G10://g10
                m_pointTime = 1e9/10000;
                break;
            case YDLIDAR_S4B://s4n
                m_pointTime = 1e9/4000;
                break;
            case YDLIDAR_S2://s2
                m_pointTime = 1e9/4000;
                break;
            case YDLIDAR_G25://g25
                if(m_sampling_rate == -1){
                    sampling_rate _rate;
                    getSamplingRate(_rate);
                    m_sampling_rate = _rate.rate;
                }
                switch(m_sampling_rate){
                case YDLIDAR_RATE_4K:
                    m_pointTime = 1e9/10000;
                    break;
                case YDLIDAR_RATE_8K:
                    m_pointTime = 1e9/16000;
                    break;
                case YDLIDAR_RATE_9K:
                    m_pointTime = 1e9/18000;
                    break;
                case YDLIDAR_RATE_10K:
                    m_pointTime = 1e9/20000;
                    break;
                }
                trans_delay = _serial->getByteTime();
                setMultipleRate(true);
                break;
            default:
                break;
            }
        }

	}


	/************************************************************************/
	/*  start to scan                                                       */
	/************************************************************************/
	result_t YDlidarDriver::startScan(bool force, uint32_t timeout ) {
		result_t ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		if (isScanning) {
			return RESULT_OK;
		}

		stop();   
		startMotor();
		checkTransTime();
        

		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(force?LIDAR_CMD_FORCE_SCAN:LIDAR_CMD_SCAN)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT) {
				return RESULT_FAIL;
			}

            if (response_header.size < 5 ) {
				return RESULT_FAIL;
			}

			ans = this->createThread();
			return ans;
		}
		return RESULT_OK;
	}

	result_t YDlidarDriver::createThread() {
		_thread = CLASS_THREAD(YDlidarDriver, cacheScanData);
		if (_thread.getHandle() == 0) {
            isScanning = false;
			return RESULT_FAIL;
		}
		isScanning = true;
		return RESULT_OK;
	}

    /************************************************************************/
    /*   startAutoScan                                                      */
    /************************************************************************/
    result_t YDlidarDriver::startAutoScan(bool force, uint32_t timeout) {
        result_t ans;
        if (!isConnected) {
            return RESULT_FAIL;
        }

        {
            ScopedLocker l(_lock);
            if ((ans = sendCommand(force?LIDAR_CMD_FORCE_SCAN:LIDAR_CMD_SCAN)) != RESULT_OK) {
                return ans;
            }

            lidar_ans_header response_header;
            if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
                return ans;
            }

            if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT) {
                return RESULT_FAIL;
            }

            if (response_header.size < 5) {
                return RESULT_FAIL;
            }

        }
        startMotor();
        return RESULT_OK;
    }


	/************************************************************************/
	/*   stop scan                                                   */
	/************************************************************************/
	result_t YDlidarDriver::stop() {
        if(isAutoconnting) {
            isAutoReconnect = false;
            isScanning = false;
            disableDataGrabbing();
            return RESULT_OK;
        }
		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
            sendCommand(LIDAR_CMD_FORCE_STOP);
			sendCommand(LIDAR_CMD_STOP);
		}

		stopMotor();
		
		return RESULT_OK;
	}

	/************************************************************************/
	/*  reset device                                                        */
	/************************************************************************/
	result_t YDlidarDriver::reset(uint32_t timeout) {
		UNUSED(timeout);
		result_t ans;
		if (!isConnected){
			return RESULT_FAIL;
		}
		ScopedLocker l(_lock);
		if ((ans = sendCommand(LIDAR_CMD_RESET))!= RESULT_OK) {
			return ans;
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/* get the current scan frequency of lidar                              */
	/************************************************************************/
	result_t YDlidarDriver::getScanFrequency(scan_frequency & frequency, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}

		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_GET_AIMSPEED)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 4) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}

			getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/* add the scan frequency by 1Hz each time                              */
	/************************************************************************/
	result_t YDlidarDriver::setScanFrequencyAdd(scan_frequency & frequency, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}

		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_ADD)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 4) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}

			getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/* decrease the scan frequency by 1Hz each time                         */
	/************************************************************************/
	result_t YDlidarDriver::setScanFrequencyDis(scan_frequency & frequency, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}

		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_DIS)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 4) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}

			getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/* add the scan frequency by 0.1Hz each time                            */
	/************************************************************************/
	result_t YDlidarDriver::setScanFrequencyAddMic(scan_frequency & frequency, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}

		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_ADDMIC)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 4) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}

			getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/* decrease the scan frequency by 0.1Hz each time                       */
	/************************************************************************/
	result_t YDlidarDriver::setScanFrequencyDisMic(scan_frequency & frequency, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}

		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_DISMIC)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 4) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}

			getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/*  get the sampling rate of lidar                                      */
	/************************************************************************/
	result_t YDlidarDriver::getSamplingRate(sampling_rate & rate, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}

		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_GET_SAMPLING_RATE)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}

			getData(reinterpret_cast<uint8_t *>(&rate), sizeof(rate));
            m_sampling_rate=rate.rate;
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/*  the set to sampling rate                                            */
	/************************************************************************/
	result_t YDlidarDriver::setSamplingRate(sampling_rate & rate, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_SET_SAMPLING_RATE)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&rate), sizeof(rate));
            m_sampling_rate=rate.rate;
		}
		return RESULT_OK;
	}

	std::string YDlidarDriver::getSDKVersion(){
		return SDKVerision;
	}


	result_t YDlidarDriver::setRotationPositive(scan_rotation & roation, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_RUN_POSITIVE)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&roation), sizeof(roation));
		}
		return RESULT_OK;
	}

	result_t YDlidarDriver::setRotationInversion(scan_rotation & roation, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_RUN_INVERSION)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&roation), sizeof(roation));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/* enable lower power                                                   */
	/************************************************************************/
	result_t YDlidarDriver::enableLowerPower(function_state & state, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_ENABLE_LOW_POWER)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&state), sizeof(state));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/*  disable lower power                                                 */
	/************************************************************************/
	result_t YDlidarDriver::disableLowerPower(function_state & state, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_DISABLE_LOW_POWER)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&state), sizeof(state));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/* get the state of motor                                               */
	/************************************************************************/
	result_t YDlidarDriver::getMotorState(function_state & state, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_STATE_MODEL_MOTOR)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&state), sizeof(state));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/* enable constant frequency                                            */
	/************************************************************************/
	result_t YDlidarDriver::enableConstFreq(function_state & state, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_ENABLE_CONST_FREQ)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&state), sizeof(state));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/* disable constant frequency                                           */
	/************************************************************************/
	result_t YDlidarDriver::disableConstFreq(function_state & state, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_DISABLE_CONST_FREQ)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&state), sizeof(state));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/*  the save current low exposure value for S4 which has signal quality                */
	/************************************************************************/
	result_t YDlidarDriver::setSaveLowExposure(scan_exposure& low_exposure, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_SAVE_SET_EXPOSURE)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&low_exposure), sizeof(low_exposure));
		}
		return RESULT_OK;


	}

	/************************************************************************/
	/*  the set to low exposure for S4 which has signal quality                */
	/************************************************************************/
	result_t YDlidarDriver::setLowExposure(scan_exposure& low_exposure, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_SET_LOW_EXPOSURE)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&low_exposure), sizeof(low_exposure));
		}
		return RESULT_OK;


	}

	/************************************************************************/
	/*  add scan exposure for S4 which has signal quality                      */
	/************************************************************************/
	result_t YDlidarDriver::setLowExposureAdd(scan_exposure & exposure, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_ADD_EXPOSURE)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&exposure), sizeof(exposure));
		}
		return RESULT_OK;

	}

	/************************************************************************/
	/*  decrease scan exposure for S4 which has signal quality                 */
	/************************************************************************/
	result_t YDlidarDriver::setLowExposurerDis(scan_exposure & exposure, uint32_t timeout)
	{
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_DIS_EXPOSURE)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&exposure), sizeof(exposure));
		}
		return RESULT_OK;


	}

	/************************************************************************/
	/*  set a circle of data fixed points for S4                  */
	/************************************************************************/
       result_t YDlidarDriver::setPointsForOneRingFlag(scan_points& points,uint32_t timeout)
       {
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
			ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_SET_SETPOINTSFORONERINGFLAG)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&points), sizeof(points));
		}
		return RESULT_OK;

       }

}
