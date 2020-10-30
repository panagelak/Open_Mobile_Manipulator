YDLIDAR SDK PACKAGE V1.3.9
=====================================================================

SDK [test](https://github.com/yangfuyuan/sdk) application for YDLIDAR

Visit EAI Website for more details about [YDLIDAR](http://www.ydlidar.com/) .

How to build YDLIDAR SDK samples
=====================================================================
    $ git clone https://github.com/yangfuyuan/sdk
    $ cd sdk
    $ git checkout master
    $ cd ..
    $ mkdir build
    $ cd build
    $ cmake ../sdk
    $ make			###linux
    $ vs open Project.sln	###windows
    
How to run YDLIDAR SDK samples
=====================================================================
    $ cd samples

linux:

	$ ./ydlidar_test
	$Lidar[ydlidar7] detected, whether to select current radar(yes/no)?:yes
	0. 115200
	1. 128000
	2. 153600
	3. 230400
	4. 512000
	$Please enter the lidar serial baud rate:2
	0. false
	1. true
	$Please enter the lidar intensity:1


windows:

	$ ydlidar_test.exe
	$Lidar[ydlidar7] detected, whether to select current radar(yes/no)?:yes
	0. 115200
	1. 128000
	2. 153600
	3. 230400
	4. 512000
	$Please enter the lidar serial baud rate:2
	0. false
	1. true
	$Please enter the lidar intensity:1


You should see YDLIDAR's scan result in the console:

	[YDLidar]: YDLidar running correctly ! The health status is good
	[YDLIDAR] Connection established in [/dev/ttyUSB0]:
	Firmware version: 1.2.0
	Hardware version: 1
	Model: S4
	Serial: 2018091100006004
	[YDLidar]: [YDLIDAR INFO] Current Sampling Rate : 4K
	[YDLidar]: set EXPOSURE MODEL SUCCESS!!!
	[YDLidar]: [YDLIDAR INFO] Now YDLIDAR is scanning ......

	[YDLidar]: Scan received[1543834103116861000]: 498 ranges
	[YDLidar]: Scan received[1543834103241414000]: 497 ranges
	[YDLidar]: Scan received[1543834103468167000]: 497 ranges
	[YDLidar]: Scan received[1543834103592417000]: 494 ranges



Lidar point data structure
=====================================================================

data structure:

	struct node_info {
    	uint8_t    sync_flag;
    	uint16_t   sync_quality;//!信号质量
    	uint16_t   angle_q6_checkbit; //!测距点角度
    	uint16_t   distance_q2; //! 当前测距点距离
    	uint64_t   stamp; //! 时间戳
    	uint8_t    scan_frequence;//! 特定版本此值才有效,无效值是0, 当前扫描频率current_frequence = scan_frequence/10.0
	} __attribute__((packed)) ;

example:

	if(data[i].scan_frequence != 0) {
 		current_frequence = data[i].scan_frequence/10.0;
	}

	current_time_stamp = data[i].stamp;

	 if(m_isMultipleRate) {
		current_distance = (float)data[i].distance_q2/2000.f;
	}else {
		current_distance = (float)data[i].distance_q2/4000.f;
	}

	current_angle = ((data[i].angle_q6_checkbit>>LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);

	current_intensity = (float)(data[i].sync_quality >> LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

	###note:current_frequence = data[0].scan_frequence/10.0.

	###if the current_frequence value equals zero, it is an invalid value.

code:
        
        void ParseScan(node_info* data, const size_t& size) {

            double current_frequence, current_distance, current_angle, current_intensity;

            uint64_t current_time_stamp;

            for (size_t i = 0; i < size; i++ ) {

                if( data[i].scan_frequence != 0) {

                    current_frequence =  data[i].scan_frequence;//or current_frequence = data[0].scan_frequence

                }

		 if(m_isMultipleRate) {
			current_distance = (float)data[i].distance_q2/2000.f;
		}else {
			current_distance = (float)data[i].distance_q2/4000.f;
		}

                current_angle = ((data[i].angle_q6_checkbit>>LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);//LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT equals 8

                current_intensity = (float)(data[i].sync_quality >> LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

            }

            if (current_frequence != 0 ) {

                printf("current lidar scan frequency: %f\n", current_frequence);

            } else {

                printf("Current lidar does not support return scan frequency\n");

            }
        }





Upgrade Log
=====================================================================

2018-12-3 version:1.3.9

1.Optimized interface.

2.Getting Radar Port Lists.

2018-05-23 version:1.3.4

1.add automatic reconnection if there is an exception

2.add serial file lock.

2018-05-14 version:1.3.3

   1.add the heart function constraint.

   2.add packet type with scan frequency support.

2018-04-16 version:1.3.2

   1.add multithreading support.

2018-04-16 version:1.3.1

   1.Compensate for each laser point timestamp.

