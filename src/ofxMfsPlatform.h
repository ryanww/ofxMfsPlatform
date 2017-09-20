//
//  motionBase.hpp
//  wrvTitanicShowControl
//
//  Created by Ryan Wilkinson on 7/22/17.
//

#pragma once
#include "ofMain.h"
#include "ofxUDPManager.h"
#include "ofxNetwork.h"
#include "ofxJSON.h"
#include "mfsMotor.h"
#include "tcpCmd.h"

#define LOWBYTE(v)   ((unsigned char) (v))
#define HIGHBYTE(v)  ((unsigned char) (((unsigned int) (v)) >> 8))
#define HIGHBYTE2(v)  ((unsigned char) (((unsigned int) (v)) >> 16))
#define HIGHBYTE3(v)  ((unsigned char) (((unsigned int) (v)) >> 24))

#define OFX_PLATFORM_STATE_DISABLED 0
#define OFX_PLATFORM_STATE_OFFLINE 1
#define OFX_PLATFORM_STATE_CONNECTION_ATTEMPT 2
#define OFX_PLATFORM_STATE_SENDING_CONFIG 3
#define OFX_PLATFORM_STATE_STANDBY 4
#define OFX_PLATFORM_STATE_RUNNING 5
#define OFX_PLATFORM_STATE_DRIVE_DISABLE 6
#define OFX_PLATFORM_STATE_FAULT 7

#define OFX_MOTION_STATE_STOP 0
#define OFX_MOTION_STATE_RUN 1

class ofxMfsPlatform : public ofThread {
    
public:
    ofxMfsPlatform();
    ~ofxMfsPlatform();
    
    //Setup
    void setup(string _configFile);
    
    //Sets
    void setTargetPosition(float _pitch, float _roll, float _heave,
                            float _sway, float _surge, float _yaw);
    void setEnabled(bool _enable);
    void resetErrors();
    void setMotionState(bool _enable);
    
    //Gets
    float getLimitPitchMin();
    float getLimitPitchMax();
    float getLimitRollMin();
    float getLimitRollMax();
    float getLimitHeaveMin();
    float getLimitHeaveMax();
    float getLimitSwayMin();
    float getLimitSwayMax();
    float getLimitSurgeMin();
    float getLimitSurgeMax();
    float getLimitYawMin();
    float getLimitYawMax();
    float getTargetPosPitch();
    float getTargetPosRoll();
    float getTargetPosHeave();
    float getTargetPosSway();
    float getTargetPosSurge();
    float getTargetPosYaw();
    bool getIsEnabled();
    bool getIsStandby();
    bool getIsRunning();
    bool getIsDriveDisabled();
    bool getIsFault();
    string getMotorStatus(int _motor);
    int getPlatformModuleState();
    string getPlatformModuleStateAsString();
    mfsMotor * getMotor(int _motor);
    long getUptime();
    
    //Events
    ofEvent<int> platformModuleStateChanged;
    
private:
    //General
    string mbIP;
    int mbUdpPort;
    int mbTcpPort;
    int platformModuleState;
    bool enableComs;
    bool tcpConnected;
    bool configFileLoaded;
    bool configSentToPlatform;
    unsigned long long lastReceivedPacketTime;
    
    //Coms
    ofxUDPManager mbUdpRx, mbUdpTx;
    ofxTCPClient tcp;
    void initUDPComs();
    void initTCPComs();
    void takePlatformOffline();
    void parseStatusPacket(char _rxMsg[1000]);
    void parseRealtimePacket(char _rxMsg[1000]);
    unsigned long long lastPosPacketTxTime;
    int posTxWait;
    vector<tcpCmd * > tcpCmdsToSend;
    unsigned long long lastTcpPacketTxTime;
    int tcpTxWait;
    
    //Platform Status
    unsigned int motionControllerState;
    unsigned long uptimeCounter;
    
    //Motion Position
    float targetPosPitch;
    float targetPosRoll;
    float targetPosHeave;
    float targetPosSway;
    float targetPosSurge;
    float targetPosYaw;
    signed int targetPosPitchInt;
    signed int targetPosRollInt;
    signed int targetPosHeaveInt;
    signed int targetPosSwayInt;
    signed int targetPosSurgeInt;
    signed int targetPosYawInt;
    
    //Motion Limits
    float pitchMin;
    float pitchMax;
    float rollMin;
    float rollMax;
    float heaveMin;
    float heaveMax;
    float swayMin;
    float swayMax;
    float surgeMin;
    float surgeMax;
    float yawMin;
    float yawMax;
    
    //Platform Config
    bool loadConfigFile(string _file);
    ofxJSONElement cfg;
    void generateConfigPackets();
    
    //Internal Functions
    void updatePlatformStatus();
    void changePlatformStatus(int _newState);
    
    //Motor Drives
    vector<mfsMotor *> motors;
    
    //Run Function
    void threadedFunction();
};

inline unsigned int toUI(const char * s){
    unsigned char msb = s[0];
    unsigned char lsb = s[1];
    return (msb<<8u)|lsb;
}

inline unsigned long toUL(const char * s){
    unsigned char b1 = s[0];
    unsigned char b2 = s[1];
    unsigned char b3 = s[2];
    unsigned char b4 = s[3];
    return (b1<<8u*3)|(b2<<8u*2)|(b3<<8u)|b4;
}

inline signed long toSL(const char * s){
    unsigned char b1 = s[0];
    unsigned char b2 = s[1];
    unsigned char b3 = s[2];
    unsigned char b4 = s[3];
    return (b1<<8u*3)|(b2<<8u*2)|(b3<<8u)|b4;
}


