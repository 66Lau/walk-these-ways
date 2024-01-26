#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <thread>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>
#include <lcm/lcm-cpp.hpp>

#include "unitree_joystick.h"
#include "state_estimator_lcmt.hpp"
#include "leg_control_data_lcmt.hpp"
#include "pd_tau_targets_lcmt.hpp"
#include "rc_command_lcmt.hpp"

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

class Custom
{
public:
    explicit Custom(){}
    ~Custom(){}

    void Init();
    void InitRobotStateClient();
    int queryServiceStatus(const std::string& serviceName);
    void activateService(const std::string& serviceName,int activate);
    void _simpleLCMThread();
    std::thread _simple_LCM_thread;
    void handleActionLCM(const lcm::ReceiveBuffer *rbuf, const std::string & chan, const pd_tau_targets_lcmt * msg);
private:
    void InitLowCmd();
    void LowStateMessageHandler(const void* messages);
    void LowCmdWrite();
 
private:
    float Kp = 60.0;
    float Kd = 5.0;
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.002; // 0.001~0.01

    RobotStateClient rsc;

    unitree_go::msg::dds_::LowCmd_ low_cmd{};      // default init
    unitree_go::msg::dds_::LowState_ low_state{};  // default init
    state_estimator_lcmt body_state_simple = {0};
    leg_control_data_lcmt joint_state_simple = {0};
    pd_tau_targets_lcmt joint_command_simple = {0};
    rc_command_lcmt rc_command = {0};
    bool _firstCommandReceived;

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    // subscribe low level states feedback
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

    /*LowCmd write thread*/
    ThreadPtr lowCmdWriteThreadPtr;

    //mem to place rc_command
    xRockerBtnDataStruct _keyData;

    lcm::LCM _simpleLCM;

    float _targetPos_1[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                              -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};

    float _targetPos_2[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                              0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

    float _targetPos_3[12] = {-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
                              -0.5, 1.36, -2.65, 0.5, 1.36, -2.65};

    float _startPos[12];
    float _duration_1 = 500;   
    float _duration_2 = 500; 
    float _duration_3 = 1000;   
    float _duration_4 = 900;   
    float _percent_1 = 0;    
    float _percent_2 = 0;    
    float _percent_3 = 0;    
    float _percent_4 = 0;    

    bool firstRun = true;
    bool done = false;
    int mode = 0;
    
};

uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void Custom::Init()
{
    _simpleLCM.subscribe("pd_plustau_targets", &Custom::handleActionLCM, this);
    _simple_LCM_thread = std::thread(&Custom::_simpleLCMThread, this);
    InitLowCmd();

    /*create publisher*/
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);

    /*loop publishing thread，2000是线程的执行时间间隔，单位为微秒，此处即为2ms*/
    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, 2000, &Custom::LowCmdWrite, this);
}

void Custom::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for(int i=0; i<20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01);   // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }
}

void Custom::InitRobotStateClient()
{
    rsc.SetTimeout(10.0f); 
    rsc.Init();
}

//查询是否在运行主运控模式
int Custom::queryServiceStatus(const std::string& serviceName)
{
    std::vector<ServiceState> serviceStateList;
    int ret,serviceStatus;
    ret = rsc.ServiceList(serviceStateList);
    size_t i, count=serviceStateList.size();
    for (i=0; i<count; i++)
    {
        const ServiceState& serviceState = serviceStateList[i];
        if(serviceState.name == serviceName)
        {
            if(serviceState.status == 0)
            {
                std::cout << "name: " << serviceState.name <<" is activate"<<std::endl;
                serviceStatus = 1;
            }
            else
            {
                std::cout << "name:" << serviceState.name <<" is deactivate"<<std::endl;
                serviceStatus = 0;
            } 
        }    
    }
    return serviceStatus;
    
}

void Custom::activateService(const std::string& serviceName,int activate)
{
    rsc.ServiceSwitch(serviceName, activate);  
}

void Custom::_simpleLCMThread(){
    while(true){
        _simpleLCM.handle();
    }
}

void Custom::handleActionLCM(const lcm::ReceiveBuffer *rbuf, const std::string & chan, const pd_tau_targets_lcmt * msg){
    (void) rbuf;
    (void) chan;

    joint_command_simple = *msg;
    _firstCommandReceived = true;
}

//此处是接受low_state消息的回调函数,from dds
void Custom::LowStateMessageHandler(const void* message)
{
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
    for(int i = 0; i < 12; i++){
        joint_state_simple.q[i] = low_state.motor_state()[i].q();
        joint_state_simple.qd[i] = low_state.motor_state()[i].dq();
        joint_state_simple.tau_est[i] = low_state.motor_state()[i].tau_est();

        // std::cout << "low_state.motor_state,q: [" << i <<"]" << low_state.motor_state()[i].q() << " " <<std::endl;
        // std::cout << "low_state.motor_state,q: [" << i <<"]" << low_state.motor_state()[i].dq() << " " <<std::endl;
        // std::cout << "low_state.motor_state,q: [" << i <<"]" << low_state.motor_state()[i].tau_est() << " " <<std::endl;
    }

    for(int i = 0; i < 4; i++){
        body_state_simple.quat[i] = low_state.imu_state().quaternion()[i];
        // std::cout << "low_state.imu_state.quaternion: [" << i <<"]" << low_state.imu_state().quaternion()[i] << " " <<std::endl;
    }
    for(int i = 0; i < 3; i++){
        body_state_simple.rpy[i] = low_state.imu_state().rpy()[i];
        body_state_simple.aBody[i] = low_state.imu_state().accelerometer()[i];
        body_state_simple.omegaBody[i] = low_state.imu_state().gyroscope()[i];
        // std::cout << "low_state.imu_state.rpy: [" << i <<"]" << low_state.imu_state().rpy()[i] << " " <<std::endl;
        // std::cout << "low_state.imu_state.accelerometer: [" << i <<"]" << low_state.imu_state().accelerometer()[i] << " " <<std::endl;
        // std::cout << "low_state.imu_state.gyroscope: [" << i <<"]" << low_state.imu_state().gyroscope()[i]<< " " <<std::endl;
    }
    for(int i = 0; i < 4; i++){
        body_state_simple.contact_estimate[i] = low_state.foot_force()[i];
        // std::cout << "low_state.foot_force: [" << i <<"]" << low_state.foot_force()[i]<< " " <<std::endl;
    }
    
    memcpy(&_keyData, &low_state.wireless_remote(), 40);
    rc_command.left_stick[0] = _keyData.lx;
    rc_command.left_stick[1] = _keyData.ly;
    rc_command.right_stick[0] = _keyData.rx;
    rc_command.right_stick[1] = _keyData.ry;
    rc_command.right_lower_right_switch = _keyData.btn.components.R2;
    rc_command.right_upper_switch = _keyData.btn.components.R1;
    rc_command.left_lower_left_switch = _keyData.btn.components.L2;
    rc_command.left_upper_switch = _keyData.btn.components.L1;


    if(_keyData.btn.components.A > 0){
        mode = 0;
    } else if(_keyData.btn.components.B > 0){
        mode = 1;
    }else if(_keyData.btn.components.X > 0){
        mode = 2;
    }else if(_keyData.btn.components.Y > 0){
        mode = 3;
    }else if(_keyData.btn.components.up > 0){
        mode = 4;
    }else if(_keyData.btn.components.right > 0){
        mode = 5;
    }else if(_keyData.btn.components.down > 0){
        mode = 6;
    }else if(_keyData.btn.components.left > 0){
        mode = 7;
    }

    // std::cout << "r2 switch =  " << rc_command.right_lower_right_switch << " " <<std::endl;
    // std::cout << "_keyData.lx =  " << rc_command.left_stick[0] << " " <<std::endl;

    rc_command.mode = mode;
}

void Custom::LowCmdWrite()
{
    
    motiontime++;
    if(motiontime>=500)
    {
        if(firstRun)
        {
            std::cout << "deployment started!  " <<std::endl;

            for(int i = 0; i < 12; i++)
            {
                low_cmd.motor_cmd()[i].q()= low_state.motor_state()[i].q();;
            }
            firstRun = false;
        }

        _simpleLCM.publish("state_estimator_data", &body_state_simple);
        _simpleLCM.publish("leg_control_data", &joint_state_simple);
        _simpleLCM.publish("rc_command", &rc_command);

        for(int i = 0; i < 12; i++){
        low_cmd.motor_cmd()[i].q() = joint_command_simple.q_des[i];
        low_cmd.motor_cmd()[i].dq() = joint_command_simple.qd_des[i];
        low_cmd.motor_cmd()[i].kp() = joint_command_simple.kp[i];
        low_cmd.motor_cmd()[i].kd() = joint_command_simple.kd[i];
        low_cmd.motor_cmd()[i].tau() = joint_command_simple.tau_ff[i];
    }

        low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
    
        lowcmd_publisher->Write(low_cmd);
    }
   
}

int main(int argc, const char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1); 
    }

    std::cout << "WARNING: Make sure the robot is hung up or lying on the ground." << std::endl
            << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ChannelFactory::Instance()->Init(0, argv[1]);

    Custom custom;
    custom.InitRobotStateClient();
    while(custom.queryServiceStatus("sport_mode"))
    {
        std::cout<<"Try to deactivate the service: "<<"sport_mode"<<std::endl;
        custom.activateService("sport_mode",0);
        sleep(1);
    }
    custom.Init();
  
    
    while (1)
    {
        sleep(10);
    }

    return 0;
}
