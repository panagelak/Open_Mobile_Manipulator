#pragma once
#include "v8stdint.h"
#include <vector>

#if !defined(_countof)
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifndef M_PI
#define M_PI 3.1415926
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

#define LIDAR_CMD_STOP                      0x65
#define LIDAR_CMD_SCAN                      0x60
#define LIDAR_CMD_FORCE_SCAN                0x61
#define LIDAR_CMD_RESET                     0x80
#define LIDAR_CMD_FORCE_STOP                0x00
#define LIDAR_CMD_GET_EAI                   0x55
#define LIDAR_CMD_GET_DEVICE_INFO           0x90
#define LIDAR_CMD_GET_DEVICE_HEALTH         0x92
#define LIDAR_ANS_TYPE_DEVINFO              0x4
#define LIDAR_ANS_TYPE_DEVHEALTH            0x6
#define LIDAR_CMD_SYNC_BYTE                 0xA5
#define LIDAR_CMDFLAG_HAS_PAYLOAD           0x80
#define LIDAR_ANS_SYNC_BYTE1                0xA5
#define LIDAR_ANS_SYNC_BYTE2                0x5A
#define LIDAR_ANS_TYPE_MEASUREMENT          0x81
#define LIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define LIDAR_RESP_MEASUREMENT_SYNC_QUALITY_SHIFT  8
#define LIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1
#define LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT    8
#define LIDAR_RESP_MEASUREMENT_DISTANCE_SHIFT  2
#define LIDAR_RESP_MEASUREMENT_DISTANCE_HALF_SHIFT 1

#define LIDAR_CMD_RUN_POSITIVE             0x06
#define LIDAR_CMD_RUN_INVERSION            0x07
#define LIDAR_CMD_SET_AIMSPEED_ADDMIC      0x09
#define LIDAR_CMD_SET_AIMSPEED_DISMIC      0x0A
#define LIDAR_CMD_SET_AIMSPEED_ADD         0x0B
#define LIDAR_CMD_SET_AIMSPEED_DIS         0x0C
#define LIDAR_CMD_GET_AIMSPEED             0x0D

#define LIDAR_CMD_SET_SAMPLING_RATE        0xD0
#define LIDAR_CMD_GET_SAMPLING_RATE        0xD1
#define LIDAR_STATUS_OK                    0x0
#define LIDAR_STATUS_WARNING               0x1
#define LIDAR_STATUS_ERROR                 0x2

#define LIDAR_CMD_ENABLE_LOW_POWER         0x01
#define LIDAR_CMD_DISABLE_LOW_POWER        0x02
#define LIDAR_CMD_STATE_MODEL_MOTOR        0x05
#define LIDAR_CMD_ENABLE_CONST_FREQ        0x0E
#define LIDAR_CMD_DISABLE_CONST_FREQ       0x0F

#define LIDAR_CMD_SAVE_SET_EXPOSURE         0x94
#define LIDAR_CMD_SET_LOW_EXPOSURE          0x95
#define LIDAR_CMD_ADD_EXPOSURE       	    0x96
#define LIDAR_CMD_DIS_EXPOSURE       	    0x97

#define LIDAR_CMD_SET_HEART_BEAT        0xD9
#define LIDAR_CMD_SET_SETPOINTSFORONERINGFLAG  0xae

#define PackageSampleMaxLngth 0x100
typedef enum {
    CT_Normal = 0,
    CT_RingStart  = 1,
    CT_Tail,
}CT;
#define Node_Default_Quality (10)
#define Node_Sync 1
#define Node_NotSync 2
#define PackagePaidBytes 10
#define PH 0x55AA

#if defined(_WIN32)
#pragma pack(1)
#endif

struct node_info {
    uint8_t    sync_flag;
    uint16_t   sync_quality;//!信号质量
    uint16_t   angle_q6_checkbit; //!测距点角度
    uint16_t   distance_q2; //! 当前测距点距离
    uint64_t   stamp; //! 时间戳
    uint8_t    scan_frequence;//! 特定版本此值才有效,无效值是0, 当前扫描频率current_frequence = scan_frequence/10.0
} __attribute__((packed)) ;

struct PackageNode {
    uint8_t PakageSampleQuality;
    uint16_t PakageSampleDistance;
}__attribute__((packed));

struct node_package {
    uint16_t  package_Head;
    uint8_t   package_CT;
    uint8_t   nowPackageNum;
    uint16_t  packageFirstSampleAngle;
    uint16_t  packageLastSampleAngle;
    uint16_t  checkSum;
    PackageNode  packageSample[PackageSampleMaxLngth];
} __attribute__((packed)) ;

struct node_packages {
    uint16_t  package_Head;
    uint8_t   package_CT;
    uint8_t   nowPackageNum;
    uint16_t  packageFirstSampleAngle;
    uint16_t  packageLastSampleAngle;
    uint16_t  checkSum;
    uint16_t  packageSampleDistance[PackageSampleMaxLngth];
} __attribute__((packed)) ;


struct device_info{
    uint8_t   model; ///< 雷达型号
    uint16_t  firmware_version; ///< 固件版本号
    uint8_t   hardware_version; ///< 硬件版本号
    uint8_t   serialnum[16];    ///< 系列号
} __attribute__((packed)) ;

struct device_health {
    uint8_t   status; ///< 健康状体
    uint16_t  error_code; ///< 错误代码
} __attribute__((packed))  ;

struct sampling_rate {
    uint8_t rate;	///< 采样频率
} __attribute__((packed))  ;

struct scan_frequency {
    uint32_t frequency;	///< 扫描频率
} __attribute__((packed))  ;

struct scan_rotation {
    uint8_t rotation;
} __attribute__((packed))  ;

struct scan_exposure {
    uint8_t exposure;	///< 低光功率模式
} __attribute__((packed))  ;

struct scan_heart_beat {
    uint8_t enable;	///< 掉电保护状态
} __attribute__((packed));

struct scan_points {
    uint8_t flag;
} __attribute__((packed))  ;

struct function_state {
    uint8_t state;
} __attribute__((packed))  ;

struct cmd_packet {
    uint8_t syncByte;
    uint8_t cmd_flag;
    uint8_t size;
    uint8_t data;
} __attribute__((packed)) ;

struct lidar_ans_header {
    uint8_t  syncByte1;
    uint8_t  syncByte2;
    uint32_t size:30;
    uint32_t subType:2;
    uint8_t  type;
} __attribute__((packed));

#if defined(_WIN32)
#pragma pack()
#endif

//! A struct for returning configuration from the YDLIDAR
struct LaserConfig {
    //! Start angle for the laser scan [rad].  0 is forward and angles are measured clockwise when viewing YDLIDAR from the top.
    float min_angle;
    //! Stop angle for the laser scan [rad].   0 is forward and angles are measured clockwise when viewing YDLIDAR from the top.
    float max_angle;
    //! Scan resolution [rad].
    float ang_increment;
    //! Scan resoltuion [s]
    float time_increment;
    //! Time between scans
    float scan_time;
    //! Minimum range [m]
    float min_range;
    //! Maximum range [m]
    float max_range;
    //! Range Resolution [m]
    float range_res;
};


//! A struct for returning laser readings from the YDLIDAR
//! currentAngle = min_angle + ang_increment*index
//! for( int i =0; i < ranges.size(); i++) {
//!     double currentAngle = config.min_angle + i*config.ang_increment;
//!     double currentDistance = ranges[i];
//! }
//!
//!
//!
struct LaserScan {
    //! Array of ranges
    std::vector<float> ranges;
    //! Array of intensities
    std::vector<float> intensities;
    //! Self reported time stamp in nanoseconds
    uint64_t self_time_stamp;
    //! System time when first range was measured in nanoseconds
    uint64_t system_time_stamp;
    //! Configuration of scan
    LaserConfig config;
};

