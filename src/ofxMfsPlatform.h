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

#define LOWBYTE(v)   ((unsigned char) (v))
#define HIGHBYTE(v)  ((unsigned char) (((unsigned int) (v)) >> 8))
#define HIGHBYTE2(v)  ((unsigned char) (((unsigned int) (v)) >> 16))
#define HIGHBYTE3(v)  ((unsigned char) (((unsigned int) (v)) >> 24))

#define OFX_PLATFORM_STATE_OFFLINE 0
#define OFX_PLATFORM_STATE_WAITING 1
#define OFX_PLATFORM_STATE_SENDING_CONFIG 2
#define OFX_PLATFORM_STATE_READY 3
#define OFX_PLATFORM_STATE_RUNNING 4
#define OFX_PLATFORM_STATE_FAULT 5


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
    string getMotorStatus(int _motor);
    string getPlatformStatus();
    
private:
    //General
    string mbIP;
    int mbPort;
    int platformModuleState;
    bool enableComs;
    bool allCfgElementsLoaded;
    
    //Coms
    ofxUDPManager mbUdpRx, mbUdpTx;
    ofxTCPClient tcp;
    void initUDPComs();
    void initTCPComs();
    void takePlatformOffline();
    void parseStatusPacket(char _rxMsg[1000]);
    void parseRealtimePacket(char _rxMsg[1000]);
    long lastPosPacketTxTime = 0;
    int posTxWait = 1000;
    
    
    //Platform Status
    unsigned int platformStatus = 0;
    unsigned long uptimeCounter = 0;
    
    //Motion Position
    float targetPosPitch = 0.0;
    float targetPosRoll = 0.0;
    float targetPosHeave = 0.0;
    float targetPosSway = 0.0;
    float targetPosSurge = 0.0;
    float targetPosYaw = 0.0;
    signed long targetPosPitchInt = 0;
    signed long targetPosRollInt = 0;
    signed long targetPosHeaveInt = 0;
    signed long targetPosSwayInt = 0;
    signed long targetPosSurgeInt = 0;
    signed long targetPosYawInt = 0;
    
    //Motion Limits
    float pitchMin = 0;
    float pitchMax = 0;
    float rollMin = 0;
    float rollMax = 0;
    float heaveMin = 0;
    float heaveMax = 0;
    float swayMin = 0;
    float swayMax = 0;
    float surgeMin = 0;
    float surgeMax = 0;
    float yawMin = 0;
    float yawMax = 0;
    
    //Platform Config
    bool loadConfigFile(string _file);
    ofxJSONElement cfg;
    void generateConfigPacket();
    vector<unsigned char *> configParamsToSend;
    
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

