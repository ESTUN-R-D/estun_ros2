#ifndef ERI_PARAM_MANAGER_H__
#define ERI_PARAM_MANAGER_H__
#include "EstunRobotERI.h"
#include <string>
#ifdef __linux__
#define UINT64 uint64_t
#define UINT32 uint32_t
#ifndef UINT16
#define UINT16 uint16_t
#endif
#elif WIN32
typedef unsigned short UINT16;
typedef unsigned __int64 UINT64;
typedef unsigned int UINT32;
typedef unsigned int uint32_t;
#endif
//机器人状态信息
struct RobotStatus
{
	bool isRobotError; //机器人端是否有错误
	char isStartCorrect;//是否开始纠偏
	bool isEndPoint;//机器人是否还在运动中 1表示运动中 0表示已经停下
	double worldCpos[16];//当前机器人世界坐标系下笛卡尔位置 - 顺序：[x, y, z, roll, pitch, yaw]- 单位：x,y,z = mm，roll,pitch,yaw = rad
	double jointValue[16];//当前机器人各关节值，单位rad
};
typedef void (*P_RobotStatusCallback)(RobotStatus robotStatus);
class ERISocketManager;

class ESTUNROBOTERI_API ERIParamManager
{
  public:
	ERIParamManager();
	~ERIParamManager();
	 /**
     * 设置 PC 端通信参数；仅写入本地网络参数，不代表机器人连接已建立
	 * 实际连接成功通常以 SDK 日志中 UDPSocket、ServoSocket、cmdSocket 均出现 connect success 为准
	 * @robotIp 机器人ip地址
	 * @cmdPort 命令通道端口号
	 * @servoPort servo通道端口号
	 * @statusPort 状态包通道端口号
	 * @返回值 1设置成功 0设置失败
     */
	int setCommunicationParam(const char *robotIp,UINT16 cmdPort,UINT16 servoPort,UINT16 statusPort);	 /**
     * 获取机器人连接状态
	 * @cmdStatus cmd连接状态
	 * @servoStatus servo连接状态
	 * @udpStatus udp连接状态
	 * @返回值 void
     */
	void getRobotConnStatus(char& cmdStatus,char& servoStatus,char& udpStatus);
	//cmd 命令
	 /**
     * 获取机器人错误信息
	 * @errId 错误ID
	 * @errMsg 错误消息内容
	 * @返回值 true成功，false失败
     */
	bool getCurErrMsg(int &errId, char errMsg[]);
	 /**
     * Movj命令
	 * @pos 点位信息
	 * @retun 1运动完成、0运动执行中、-1命令执行失败
     */
	int movJ(double pos[]);
	 /**
     * MovL命令
	 * @pos 点位信息
	 * @retun 1运动完成、0运动执行中、-1命令执行失败
     */
	int movL(double pos[]);
	 /**
     * 获取对机器人控制权
     */
	bool getControlRobot();
	 /**
     * 释放对机器人的控制权
     */
	bool releaseControlOverRobot();
	 /**
     * 释放对机器人的控制权
     */
	bool forceReleaseControlOverRobot();
	 /**
     * 开始实时控制
	 * @motionType 0关节 1笛卡尔
     */
	bool startServo(int motionType = 0);
	 /**
     * 设置采样周期
	 * @period 采样周期 单位ms
	 * @filterType 滤波类型 0-关闭滤波 1-平均滤波值 2-二阶低通滤波 3-椭圆滤波
     */
	bool setERIMotionParam(UINT16 period,UINT16 filterType);
	/**
     * 设置ERI纠偏参数
	 * @alarmThreshold 纠偏量报警阈值 单位mm
	 * @corrCoord 纠偏坐标系 0-世界坐标系  1-工具坐标系 2-TTS坐标系
     */
	bool setCorrectParam(double alarmThreshold[],UINT16 corrCoord);
	 /**
     * 设置Tool
	 * @toolId toolId
     */
	bool setTool(const UINT16 toolId);
	/**
     * 设置user
	 * @userId userId
     */
	bool setUser(const UINT16 userId);
	/**
     * 修改工具参数
	 * @mapTool  toolId工具id 
	 * @toolData 工具参数数据分别对应 x,y,z,a,b,c
     */
	bool modifyTool(int toolId,double toolData[6]);
	/**
     * 修改用户坐标参数
	 * @mapUser  userId用户坐标id
	 * @userData 用户坐标数据分别对应 x,y,z,a,b,c
     */
	bool modifyUser(int userId,double userData[6]);
	//end cmd 命令
	/////////////////实时命令//////////////
	 /**
     * 实时关节运动
	 * @timeStamp 时间戳
	 * @pos 点位信息
	 * [J1 ~ J6] （单位：deg）
	 * @IsEndPoint 是否结束点
	 * @return -1失败
     */
	int servoToAPOS(UINT64& timeStamp,double *pos,bool IsEndPoint = false);	 
	/**
     * 实时笛卡尔运动
	 * @timeStamp 时间戳
	 * @pos 点位信息
	 * [x, y, z, a, b, c]
     *- x,y,z 是 mm
     *- a,b,c 是 deg，并且语义是 ZYX 欧拉角
	 * @IsEndPoint 是否结束点
	 * @return -1失败
     */
	int servoToCPOS(UINT64& timeStamp,double *pos,bool IsEndPoint = false);
	/**
     * 实时偏移运动
	 * @timeStamp 时间戳
	 * @pos 点位信息
	 * [x, y, z, a, b, c]
     *- x,y,z 是 mm
     *- a,b,c 是 deg，并且语义是 ZYX 固定角
	 * @IsEndPoint 是否结束点
	 * @return -1失败
     */
	int servoToOFFSET(UINT64& timeStamp,double *pos,bool IsEndPoint = false);
	////////////////end 实时命令///////////
	/**
     * UDP状态回调函数
	 * @pfunc 函数指针,执行需要状态包回调函数
	 * 该回调函数处于异步线程,请勿阻塞
     */
	void setRobotStatusCallback(P_RobotStatusCallback pfunc);
	/**
	* 获取笛卡尔坐标系下的值
	* @pos位置信息
	* @return true成功 false失败
	*/
	bool getWorldCpos(double pos[16]);
	/**
	* 获取关节值
	* @pos关节值
	* @return true成功 false失败
	*/
	bool getJointValue(double pos[16]);
	/**
	* 设置物理DO数据
	* @port 端口号
	* @val  端口值
	* @return true成功 false失败
	*/
	bool setDo(UINT32 port,UINT32 val);
	/**
	* 获取系统DO
	*@return 系统64位DO十进制值
	*/
	UINT64 getCurrentDO();	
	/**
	* 获取系统DO
	* @port 端口号
	* return @val  端口值
	*/
	UINT32 getCurrentDO(const UINT32 port);
private:
	void reverseByte(double *points ,int len);
  private:
	  ERISocketManager *m_socketManage;

};

#endif // ERI_PARAM_MANAGER_H__
