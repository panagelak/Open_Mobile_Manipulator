#ifndef YDLIDAR_DRIVER_H
#define YDLIDAR_DRIVER_H
#include <stdlib.h>
#include <atomic>
#include <map>
#include "locker.h"
#include "serial.h"
#include "thread.h"
#include "ydlidar_protocol.h"
#include "Console.h"

#if !defined(__cplusplus)
#ifndef __cplusplus
#error "The YDLIDAR SDK requires a C++ compiler to be built"
#endif
#endif



using namespace std;
using namespace serial;

namespace ydlidar{

	class YDlidarDriver
	{
	public:
        /**
        * A constructor.
        * A more elaborate description of the constructor.
        */
         YDlidarDriver();

        /**
        * A destructor.
        * A more elaborate description of the destructor.
        */
         virtual ~YDlidarDriver();


        /**
        * @brief lidarPortList 获取雷达端口
        * @return 在线雷达列表
        */
        static std::map<std::string, std::string> lidarPortList();

		/**
		* @brief 连接雷达 \n
    	* 连接成功后，必须使用::disconnect函数关闭
    	* @param[in] port_path    串口号
    	* @param[in] fileMode    波特率，YDLIDAR雷达有以下几个波特率：
    	*     115200 F4, G4C, S4A
    	*     128000 X4
    	*     153600 S4B
    	*     230600 F4PRO, G4
    	* @return 返回连接状态
		* @retval 0     成功
    	* @retval < 0   失败
    	* @note连接成功后，必须使用::disconnect函数关闭
    	* @see 函数::YDlidarDriver::disconnect (“::”是指定有连接功能,可以看文档里的disconnect变成绿,点击它可以跳转到disconnect.)
    	*/
		result_t connect(const char * port_path, uint32_t baudrate);

		/**
		* @brief 断开雷达连接 
    	*/
		void disconnect();

		/**
		* @brief 获取当前SDK版本号 \n
    	* 静态函数
    	* @return 返回当前SKD 版本号
    	*/
		static std::string getSDKVersion();

		/**
		* @brief 扫图状态 \n
    	* @return 返回当前雷达扫图状态
		* @retval true     正在扫图
    	* @retval false    扫图关闭
    	*/
        bool isscanning() const;

		/**
		* @brief 连接雷达状态 \n
    	* @return 返回连接状态
		* @retval true     成功
    	* @retval false    失败
    	*/
        bool isconnected() const;

		/**
		* @brief 设置雷达是否带信号质量 \n
    	* 连接成功后，必须使用::disconnect函数关闭
    	* @param[in] isintensities    是否带信号质量:
		*     true	带信号质量
		*	  false 无信号质量
        * @note只有S4B(波特率是153600)雷达支持带信号质量, 别的型号雷达暂不支持
    	*/
        void setIntensities(const bool& isintensities);

        /**
         * @brief 设置雷达异常自动重新连接 \n
         * @param[in] enable    是否开启自动重连:
         *     true	开启
         *	  false 关闭
         */
        void setAutoReconnect(const bool& enable);

 		/**
         * @brief 设置雷达采样倍频 \n
         * @param[in] enable    是否开启采样倍频:
         *     true	开启
         *	  false 关闭
         */
        void setMultipleRate(const bool& enable);

		/**
		* @brief 获取当前雷达掉电保护功能 \n
		* @return 返回掉电保护是否开启
    	* @retval true     掉电保护开启
    	* @retval false    掉电保护关闭
    	*/
        bool getMultipleRate() const;

		/**
		 * @brief 检测传输时间 \n
		 * */
		void checkTransTime();

		/**
		* @brief 获取雷达设备健康状态 \n
    	* @return 返回执行结果
    	* @retval RESULT_OK       获取成功
    	* @retval RESULT_FAILE or RESULT_TIMEOUT   获取失败
    	*/
		result_t getHealth(device_health & health, uint32_t timeout = DEFAULT_TIMEOUT);

		/**
		* @brief 获取雷达设备信息 \n
		* @param[in] info     设备信息
    	* @param[in] timeout  超时时间  
    	* @return 返回执行结果
    	* @retval RESULT_OK       获取成功
    	* @retval RESULT_FAILE or RESULT_TIMEOUT   获取失败
    	*/
		result_t getDeviceInfo(device_info & info, uint32_t timeout = DEFAULT_TIMEOUT);

		/**
		* @brief 开启扫描 \n
    	* @param[in] force    扫描模式
    	* @param[in] timeout  超时时间  
    	* @return 返回执行结果
    	* @retval RESULT_OK       开启成功
    	* @retval RESULT_FAILE    开启失败
		* @note 只用开启一次成功即可
    	*/
		result_t startScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT) ;

		/**
		* @brief 关闭扫描 \n
    	* @return 返回执行结果
    	* @retval RESULT_OK       关闭成功
    	* @retval RESULT_FAILE    关闭失败
    	*/
		result_t stop();

		
		/**
		* @brief 获取激光数据 \n
    	* @param[in] nodebuffer 激光点信息
		* @param[in] count      一圈激光点数
    	* @param[in] timeout    超时时间  
    	* @return 返回执行结果
    	* @retval RESULT_OK       获取成功
    	* @retval RESULT_FAILE    获取失败
		* @note 获取之前，必须使用::startScan函数开启扫描
    	*/
		result_t grabScanData(node_info * nodebuffer, size_t & count, uint32_t timeout = DEFAULT_TIMEOUT) ;


		/**
		* @brief 补偿激光角度 \n
		* 把角度限制在0到360度之间
    	* @param[in] nodebuffer 激光点信息
		* @param[in] count      一圈激光点数
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 补偿之前，必须使用::grabScanData函数获取激光数据成功
    	*/
		result_t ascendScanData(node_info * nodebuffer, size_t count);

		/**	
		* @brief 重置激光雷达 \n
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作, 如果在扫描中调用::stop函数停止扫描
    	*/
		result_t reset(uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 打开电机 \n
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
    	*/
		result_t startMotor();

		/**	
		* @brief 关闭电机 \n
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
    	*/
		result_t stopMotor();


		/**	
		* @brief 获取激光雷达当前扫描频率 \n
		* @param[in] frequency    扫描频率
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t getScanFrequency(scan_frequency & frequency, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 设置增加扫描频率1HZ \n
		* @param[in] frequency    扫描频率
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t setScanFrequencyAdd(scan_frequency & frequency, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 设置减小扫描频率1HZ \n
		* @param[in] frequency    扫描频率
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t setScanFrequencyDis(scan_frequency & frequency, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 设置增加扫描频率0.1HZ \n
		* @param[in] frequency    扫描频率
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t setScanFrequencyAddMic(scan_frequency & frequency, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 设置减小扫描频率0.1HZ \n
		* @param[in] frequency    扫描频率
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t setScanFrequencyDisMic(scan_frequency & frequency, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 获取激光雷达当前采样频率 \n
		* @param[in] frequency    采样频率
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t getSamplingRate(sampling_rate & rate, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 设置激光雷达当前采样频率 \n
		* @param[in] frequency    采样频率
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t setSamplingRate(sampling_rate & rate, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 设置电机顺时针旋转 \n
		* @param[in] rotation    旋转方向
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t setRotationPositive(scan_rotation & rotation, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 设置电机逆顺时针旋转 \n
		* @param[in] rotation    旋转方向
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t setRotationInversion(scan_rotation & rotation, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 低功耗使能 \n
		* @param[in] state    低功耗状态
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作,低功耗关闭,关闭后 G4 在空闲模式下电\n
		* 机和测距单元仍然工作
    	*/
		result_t enableLowerPower(function_state & state, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 关闭低功耗 \n
		* @param[in] state    低功耗状态
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作,关闭后 G4 在空闲模式下电\n
		* 机和测距单元仍然工作
    	*/
		result_t disableLowerPower(function_state & state, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 获取电机状态 \n
		* @param[in] state    电机状态
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t getMotorState(function_state & state, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 开启恒频功能 \n
		* @param[in] state    	  恒频状态
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t enableConstFreq(function_state & state, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 关闭恒频功能 \n
		* @param[in] state    	  恒频状态
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t disableConstFreq(function_state & state, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 保存当前激光曝光值 \n
		* @param[in] low_exposure    低光功能状态
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作, 当前操作需在非低光功率模式下, \n
		* 只有S4雷达支持此功能
    	*/
		result_t setSaveLowExposure(scan_exposure& low_exposure, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 设置低光功率模式 \n
		* @param[in] low_exposure    扫描频率
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作, 当前操作是开关量,只有S4雷达支持此功能
    	*/
		result_t setLowExposure(scan_exposure& low_exposure, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 增加激光曝光值 \n
		* @param[in] exposure     曝光值
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作,只有S4雷达支持此功能
    	*/
		result_t setLowExposureAdd(scan_exposure & exposure, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 减小激光曝光值 \n
		* @param[in] exposure     曝光值
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作,只有S4雷达支持此功能
    	*/
		result_t setLowExposurerDis(scan_exposure & exposure, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 设置扫描一圈固定激光点数 \n
		* @param[in] points    	  固定点数状态
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作, 当前操作是开关量,只有S4雷达支持此功能
    	*/
		result_t setPointsForOneRingFlag(scan_points& points,uint32_t timeout = DEFAULT_TIMEOUT);

	protected:

		/**
		* @brief 创建解析雷达数据线程 \n
		* @note 创建解析雷达数据线程之前，必须使用::startScan函数开启扫图成功
    	*/
		result_t createThread();


        /**
         * @brief 异常自动重新连接雷达
         * @return 返回连接结果
         * @retval true     成功
         * @retval false    失败
         */
        bool autoReconnectLidar();


        /**
        * @brief 重新连接开启扫描 \n
        * @param[in] force    扫描模式
        * @param[in] timeout  超时时间
        * @return 返回执行结果
        * @retval RESULT_OK       开启成功
        * @retval RESULT_FAILE    开启失败
        * @note sdk 自动重新连接调用
        */

        result_t startAutoScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT) ;

		/**
		* @brief 解包激光数据 \n
    	* @param[in] node 解包后激光点信息
		* @param[in] timeout     超时时间
    	*/
		result_t waitPackage(node_info * node, uint32_t timeout = DEFAULT_TIMEOUT);

		/**
		* @brief 发送数据到雷达 \n
    	* @param[in] nodebuffer 激光信息指针
    	* @param[in] count      激光点数大小	
		* @param[in] timeout      超时时间	
		* @return 返回执行结果
    	* @retval RESULT_OK       成功
		* @retval RESULT_TIMEOUT  等待超时
    	* @retval RESULT_FAILE    失败	
    	*/
		result_t waitScanData(node_info * nodebuffer, size_t & count, uint32_t timeout = DEFAULT_TIMEOUT);

		/**
		* @brief 激光数据解析线程 \n
    	*/
		int cacheScanData();

		/**
		* @brief 发送数据到雷达 \n
    	* @param[in] cmd 	 命名码
    	* @param[in] payload      payload	
		* @param[in] payloadsize      payloadsize	
		* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败	
    	*/
		result_t sendCommand(uint8_t cmd, const void * payload = NULL, size_t payloadsize = 0);

		/**
		* @brief 等待激光数据包头 \n
    	* @param[in] header 	 包头
    	* @param[in] timeout      超时时间	
		* @return 返回执行结果
    	* @retval RESULT_OK       获取成功
		* @retval RESULT_TIMEOUT  等待超时
    	* @retval RESULT_FAILE    获取失败	
		* @note 当timeout = -1 时, 将一直等待
    	*/
		result_t waitResponseHeader(lidar_ans_header * header, uint32_t timeout = DEFAULT_TIMEOUT);

		/**
		* @brief 等待固定数量串口数据 \n
    	* @param[in] data_count 	 等待数据大小
    	* @param[in] timeout    	 等待时间	
		* @param[in] returned_size   实际数据大小	
		* @return 返回执行结果
    	* @retval RESULT_OK       获取成功
		* @retval RESULT_TIMEOUT  等待超时
    	* @retval RESULT_FAILE    获取失败	
		* @note 当timeout = -1 时, 将一直等待
    	*/
        result_t waitForData(size_t data_count,uint32_t timeout = DEFAULT_TIMEOUT, size_t * returned_size = NULL);

		/**
		* @brief 获取串口数据 \n
    	* @param[in] data 	 数据指针
    	* @param[in] size    数据大小	
		* @return 返回执行结果
    	* @retval RESULT_OK       获取成功
    	* @retval RESULT_FAILE    获取失败	
    	*/
		result_t getData(uint8_t * data, size_t size);

		/**
		* @brief 串口发送数据 \n
    	* @param[in] data 	 发送数据指针
    	* @param[in] size    数据大小	
		* @return 返回执行结果
    	* @retval RESULT_OK       发送成功
    	* @retval RESULT_FAILE    发送失败	
    	*/
		result_t sendData(const uint8_t * data, size_t size);

		/**
		* @brief 关闭数据获取通道 \n
    	*/
		void disableDataGrabbing();

		/**
		* @brief 设置串口DTR \n
    	*/
		void setDTR();

		/**
		* @brief 清除串口DTR \n
    	*/
		void clearDTR();


	public:
		std::atomic<bool>     isConnected;  ///< 串口连接状体
        std::atomic<bool>     isScanning;   ///< 扫图状态
        std::atomic<bool>     isAutoReconnect;  ///< 异常自动从新连接
        std::atomic<bool>     isAutoconnting; ///< 是否正在自动连接中

		enum {
			DEFAULT_TIMEOUT 	= 2000,    /**< 默认超时时间. */ 
			DEFAULT_HEART_BEAT 	= 1000, /**< 默认检测掉电功能时间. */ 
			MAX_SCAN_NODES 		= 3600,	   /**< 最大扫描点数. */ 
            DEFAULT_TIMEOUT_COUNT = 1,
		};
		enum { 
			YDLIDAR_F4			= 1, /**< F4雷达型号代号. */ 
			YDLIDAR_T1			= 2, /**< T1雷达型号代号. */ 
			YDLIDAR_F2			= 3, /**< F2雷达型号代号. */ 
			YDLIDAR_S4			= 4, /**< S4雷达型号代号. */ 
			YDLIDAR_G4			= 5, /**< G4雷达型号代号. */ 
			YDLIDAR_X4			= 6, /**< X4雷达型号代号. */ 
			YDLIDAR_G4PRO		= 7, /**< G4PRO雷达型号代号. */ 
			YDLIDAR_F4PRO		= 8, /**< F4PRO雷达型号代号. */ 
			YDLIDAR_G4C			= 9, /**< G4C雷达型号代号. */ 
			YDLIDAR_G10			= 10,/**< G10雷达型号代号. */ 
            YDLIDAR_S4B 		= 11,/**< S4B雷达型号代号. */ 
            YDLIDAR_S2 			= 12,/**< S2雷达型号代号. */ 
            YDLIDAR_G25 		= 13,/**< G25雷达型号代号. */ 
            YDLIDAR_Tail,/**< 雷达型号代号. */ 

		};

		enum {
            YDLIDAR_RATE_4K 	= 0,
            YDLIDAR_RATE_8K 	= 1,
            YDLIDAR_RATE_9K 	= 2,
            YDLIDAR_RATE_10K 	= 3,
        };

		enum { 
			YDLIDAR_F4_BAUD		= 115200, /**< F4雷达型号波特率. */ 
			YDLIDAR_T1_BAUD		= 115200, /**< T1雷达型号波特率. */ 
			YDLIDAR_F2_BAUD		= 115200, /**< F2雷达型号波特率. */ 
			YDLIDAR_S4_BAUD		= 115200, /**< S4雷达型号波特率. */ 
			YDLIDAR_G4_BAUD		= 230400, /**< G4雷达型号波特率. */ 
			YDLIDAR_X4_BAUD		= 128000, /**< X4雷达型号波特率. */ 
			YDLIDAR_G4PRO_BAUD	= 230400, /**< G4PRO雷达型号波特率. */ 
			YDLIDAR_F4PRO_BAUD	= 128000, /**< F4PRO雷达型号波特率. */ 
			YDLIDAR_G4C_BAUD	= 115200, /**< G4C雷达型号波特率. */ 
			YDLIDAR_G10_BAUD	= 230400,/**< G10雷达型号波特率. */ 
            YDLIDAR_S4B_BAUD 	= 153600,/**< S4B雷达型号波特率. */ 
            YDLIDAR_S2_BAUD 	= 115200,/**< S2雷达型号波特率. */ 
            YDLIDAR_G25_BAUD 	= 512000,/**< G25雷达型号波特率. */ 

		};

		node_info      	scan_node_buf[3600];  ///< 激光点信息
		size_t         	scan_node_count;      ///< 激光点数
		Event          	_dataEvent;			 ///< 数据同步事件
		Locker         	_lock;				///< 线程锁
        Locker 			_serial_lock;                ///< 串口锁
		Thread 	       	_thread;				///< 线程id

	private:
        int PackageSampleBytes;             ///< 一个包包含的激光点数
		serial::Serial *_serial;			///< 串口
		bool m_intensities;					///< 信号质量状体
        int m_sampling_rate;					///< 采样频率
		int model;							///< 雷达型号
        uint32_t m_baudrate;					///< 波特率
		bool isSupportMotorCtrl;			///< 是否支持电机控制
		uint64_t m_ns;						///< 时间戳
        uint64_t m_last_ns;					///< 时间戳
		uint32_t m_pointTime;				///< 激光点直接时间间隔
		uint32_t trans_delay;				///< 串口传输一个byte时间

        node_package package;
        node_packages packages;

        uint16_t package_Sample_Index;
        float IntervalSampleAngle;
        float IntervalSampleAngle_LastPackage;
        uint16_t FirstSampleAngle;
        uint16_t LastSampleAngle;
        uint16_t CheckSun;

        uint16_t CheckSunCal;
        uint16_t SampleNumlAndCTCal;
        uint16_t LastSampleAngleCal;
        bool CheckSunResult;
        uint16_t Valu8Tou16;
		bool isMultipleRate;
        uint8_t scan_frequence;

        std::string serial_port;///< 雷达端口

	};
}

#endif // YDLIDAR_DRIVER_H
