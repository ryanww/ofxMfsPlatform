//
//  motionBase.cpp
//  wrvTitanicShowControl
//
//  Created by Ryan Wilkinson on 7/22/17.
//

#include "ofxMfsPlatform.h"

//Init
#pragma mark INIT
ofxMfsPlatform::ofxMfsPlatform(){
    mbIP = "";
    mbPort = 0;
    platformModuleState = OFX_PLATFORM_STATE_OFFLINE;
    enableComs = false;
    allCfgElementsLoaded = false;
    
    //Motion Position
    targetPosPitch = 0.0;
    targetPosRoll = 0.0;
    targetPosHeave = 0.0;
    targetPosSway = 0.0;
    targetPosSurge = 0.0;
    targetPosYaw = 0.0;
    targetPosPitchInt = 0;
    targetPosRollInt = 0;
    targetPosHeaveInt = 0;
    targetPosSwayInt = 0;
    targetPosSurgeInt = 0;
    targetPosYawInt = 0;
    pitchMin = 0;
    pitchMax = 0;
    rollMin = 0;
    rollMax = 0;
    heaveMin = 0;
    heaveMax = 0;
    swayMin = 0;
    swayMax = 0;
    surgeMin = 0;
    surgeMax = 0;
    yawMin = 0;
    yawMax = 0;
    lastPosPacketTxTime = 0;
    posTxWait = 1000;
    motionControllerState = 2;
    uptimeCounter = 0;
    lastReceivedPacketTime = 0;
}
ofxMfsPlatform::~ofxMfsPlatform(){
    stopThread();
}

//Setup
#pragma mark SETUP
void ofxMfsPlatform::setup(string _configFile){
    ofLogVerbose("MOTION BASE")<<"Starting Motion Base setup";
    
    //Add internal motors
    for (int i=0; i<6; i++){
        motors.push_back(new mfsMotor());
    }
    
    //Configuration
    if (!loadConfigFile(_configFile)){
        ofLogError("MOTION BASE")<<"Error loading configuration file";
        return;
    }

    ofLogVerbose("MOTION BASE")<<"Motion base setup complete";
    startThread();
    
    allCfgElementsLoaded = true;
}
void ofxMfsPlatform::initUDPComs(){
    ofLogVerbose("MOTION BASE")<<"Initializing UDP coms";
    //Setup UDP Receiver
    mbUdpRx.Create();
    mbUdpRx.Bind(51302);
    mbUdpRx.SetNonBlocking(true);
    
    //Setup UDP Transmitter
    ofxUDPSettings settings;
    settings.sendTo(mbIP, mbPort);
    settings.blocking = false;
    mbUdpTx.Setup(settings);
}
void ofxMfsPlatform::initTCPComs(){
    ofLogVerbose("MOTION BASE")<<"Initializing TCP coms";
    bool connected = false;
    ofxTCPSettings settings(mbIP, mbPort);
    connected = tcp.setup(settings);
}
void ofxMfsPlatform::takePlatformOffline(){
    tcp.close();
    allCfgElementsLoaded = false;
//    allCfgSent = false;
}

//Sets
#pragma mark SETS
void ofxMfsPlatform::setTargetPosition(float _pitch, float _roll, float _heave,
                                    float _sway, float _surge, float _yaw){
   
    if (allCfgElementsLoaded){
        targetPosPitch = CLAMP(_pitch, pitchMin, pitchMax);
        targetPosPitchInt = (signed long)(targetPosPitch*1000);
        
        targetPosRoll = CLAMP(_roll, rollMin, rollMax);
        targetPosRollInt = (signed long)(targetPosRoll*1000);
        
        targetPosHeave = CLAMP(_heave, heaveMin, heaveMax);
        targetPosHeaveInt = (signed long)(targetPosHeave*1000);
        
        targetPosSway = CLAMP(_sway, swayMin, swayMax);
        targetPosSwayInt = (signed long)(targetPosSway*1000);
        
        targetPosSurge = CLAMP(_surge, surgeMin, surgeMax);
        targetPosSurgeInt = (signed long)(targetPosSurge*1000);
        
        targetPosYaw = CLAMP(_yaw, yawMin, yawMax);
        targetPosYawInt = (signed long)(targetPosYaw*1000);
    }
}
void ofxMfsPlatform::setEnabled(bool _enable){
    enableComs = _enable;
    //TODO: Add connect function, clear on disable..
}


//Running Thread
#pragma mark RUNNING THREAD
void ofxMfsPlatform::threadedFunction(){
    setThreadName("ofxMfsPlatform");
    while(isThreadRunning()){
        
        //UDP RX
        char udpRxMsg[1000];
        mbUdpRx.Receive(udpRxMsg,1000);
        if (udpRxMsg[0] == 0x01){
            parseStatusPacket(udpRxMsg);
            lastReceivedPacketTime = ofGetElapsedTimeMillis();
        }
        if (udpRxMsg[0] == 0x02){
            parseRealtimePacket(udpRxMsg);
            lastReceivedPacketTime = ofGetElapsedTimeMillis();
        }
        
        //TCP RX
        if (tcp.isConnected()){
            char tcpRxMsg[1000];
            tcp.receiveRawBytes(tcpRxMsg, 1000);
//            string str = tcp.receive();
//            if( str.length() > 0 ){
//                msgRx = str;
//            }
        }
            
        //Transmit Packet
        if (allCfgElementsLoaded && enableComs){
            if(ofGetElapsedTimeMillis()>= lastPosPacketTxTime+posTxWait){
                uint8_t localByteArray[27];
                localByteArray[0] = 0x07;
                localByteArray[1] = 0x12;
                localByteArray[2] = HIGHBYTE3(targetPosPitchInt);
                localByteArray[3] = HIGHBYTE2(targetPosPitchInt);
                localByteArray[4] = HIGHBYTE(targetPosPitchInt);
                localByteArray[5] = LOWBYTE(targetPosPitchInt);
                
                localByteArray[6] = HIGHBYTE3(targetPosRollInt);
                localByteArray[7] = HIGHBYTE2(targetPosRollInt);
                localByteArray[8] = HIGHBYTE(targetPosRollInt);
                localByteArray[9] = LOWBYTE(targetPosRollInt);
                
                localByteArray[10] = HIGHBYTE3(targetPosHeaveInt);
                localByteArray[11] = HIGHBYTE2(targetPosHeaveInt);
                localByteArray[12] = HIGHBYTE(targetPosHeaveInt);
                localByteArray[13] = LOWBYTE(targetPosHeaveInt);
                
                localByteArray[14] = HIGHBYTE3(targetPosSwayInt);
                localByteArray[15] = HIGHBYTE2(targetPosSwayInt);
                localByteArray[16] = HIGHBYTE(targetPosSwayInt);
                localByteArray[17] = LOWBYTE(targetPosSwayInt);
                
                localByteArray[18] = HIGHBYTE3(targetPosSurgeInt);
                localByteArray[19] = HIGHBYTE2(targetPosSurgeInt);
                localByteArray[20] = HIGHBYTE(targetPosSurgeInt);
                localByteArray[21] = LOWBYTE(targetPosSurgeInt);
                
                localByteArray[22] = HIGHBYTE3(targetPosYawInt);
                localByteArray[23] = HIGHBYTE2(targetPosYawInt);
                localByteArray[24] = HIGHBYTE(targetPosYawInt);
                localByteArray[25] = LOWBYTE(targetPosYawInt);
                
                localByteArray[26] = 0x00;
                unsigned char * t = localByteArray;
#warning disabled send!
                //mbUdpTx.Send((const char*)t, 27);
                lastReceivedPacketTime = ofGetElapsedTimeMillis();
            }
        }
        //Update status/state
        updatePlatformStatus();
    }
}

//Internal Functions
#pragma mark INTERNAL FUNCTIONS
void ofxMfsPlatform::parseStatusPacket(char _rxMsg[1000]){
    
    //Check if valid
    if (_rxMsg[0] != 0x01){
        return;
    }
    
    //Platform Status
    char tmpPs[2]; tmpPs[0] = _rxMsg[1]; tmpPs[1] = _rxMsg[2];
    motionControllerState = toUI(tmpPs);
    
    //Uptime
    char tmpUc[4]; tmpUc[0]=_rxMsg[3]; tmpUc[1]=_rxMsg[4]; tmpUc[2]=_rxMsg[5]; tmpUc[3]=_rxMsg[6];
    uptimeCounter = toUL(tmpUc);
    
    //Motor
    for (int i=0; i<6; i++){
        int offset = 31*i;
        if (motors.size()>i){
            char tmpMo[2]; tmpMo[0] = _rxMsg[7+offset]; tmpMo[1] = _rxMsg[7+offset+1];
            motors.at(i)->setMotorOverload(toUI(tmpMo));
            
            char tmpDo[2]; tmpDo[0] = _rxMsg[9+offset]; tmpDo[1] = _rxMsg[9+offset+1];
            motors.at(i)->setDriveOverload(toUI(tmpDo));
            
            char tmpPsT[2]; tmpPsT[0] = _rxMsg[11+offset]; tmpPsT[1] = _rxMsg[11+offset+1];
            motors.at(i)->setDriveTemp(toUI(tmpPsT));
            
            char tmpMt[2]; tmpMt[0] = _rxMsg[13+offset]; tmpMt[1] = _rxMsg[13+offset+1];
            motors.at(i)->setMotorTemp(toUI(tmpMt));
            
            char tmpMxPsOl[2]; tmpMxPsOl[0] = _rxMsg[15+offset]; tmpMxPsOl[1] = _rxMsg[15+offset+1];
            motors.at(i)->setDriveMaxOverload(toUI(tmpMxPsOl));
            
            char tmpMOl[2]; tmpMOl[0] = _rxMsg[17+offset]; tmpMOl[1] = _rxMsg[17+offset+1];
            motors.at(i)->setMotorOverload(toUI(tmpMOl));
            
            char tmpLw[2]; tmpLw[0] = _rxMsg[19+offset]; tmpLw[1] = _rxMsg[19+offset+1];
            motors.at(i)->setDriveLastWarning(toUI(tmpLw));
            
            char tmpLe[2]; tmpLe[0] = _rxMsg[21+offset]; tmpLe[1] = _rxMsg[21+offset+1];
            motors.at(i)->setDriveLastError(toUI(tmpLe));
            
            char tmpWl[4];
            tmpWl[0] = _rxMsg[23+offset]; tmpWl[1] = _rxMsg[23+offset+1];
            tmpWl[2] = _rxMsg[23+offset+2]; tmpWl[3] = _rxMsg[23+offset+3];
            motors.at(i)->setDriveLastWarning(toUL(tmpWl));
            
            char tmpSl[4];
            tmpSl[0] = _rxMsg[27+offset]; tmpSl[1] = _rxMsg[27+offset+1];
            tmpSl[2] = _rxMsg[27+offset+2]; tmpSl[3] = _rxMsg[27+offset+3];
            motors.at(i)->setDriveSigLatched(toUL(tmpSl));
            
            char tmpOp[2]; tmpOp[0] = _rxMsg[31+offset]; tmpOp[1] = _rxMsg[31+offset+1];
            motors.at(i)->setDriveOutputPower(toUI(tmpOp));
            
            char tmpMOp[2]; tmpMOp[0] = _rxMsg[33+offset]; tmpMOp[1] = _rxMsg[33+offset+1];
            motors.at(i)->setDriveOutputPowerMean(toUI(tmpMOp));
            
            char tmpDcBV[2]; tmpDcBV[0] = _rxMsg[35+offset]; tmpDcBV[1] = _rxMsg[35+offset+1];
            motors.at(i)->setDriveDcBusVoltage(toUI(tmpDcBV));
            
            char tmpMs[2]; tmpMs[0] = 0x00; tmpMs[1] = _rxMsg[37+offset];
            motors.at(i)->setMotorStatus(toUI(tmpMs));
        }
    }
}
void ofxMfsPlatform::parseRealtimePacket(char _rxMsg[1000]){
    //Check if valid
    if (_rxMsg[0] != 0x02){
        return;
    }
    
    //Motor
    for (int i=0; i<6; i++){
        int offset = 12*i;
        if (motors.size()>i){
            char tmpMol[2]; tmpMol[0] = _rxMsg[1+offset]; tmpMol[1] = _rxMsg[1+offset+1];
            motors.at(i)->setMotorLoad(toUI(tmpMol));
            
            char tmpDol[2]; tmpDol[0] = _rxMsg[3+offset]; tmpDol[1] = _rxMsg[3+offset+1];
            motors.at(i)->setDriveLoad(toUI(tmpDol));
            
            char tmpOP[2]; tmpOP[0] = _rxMsg[5+offset]; tmpOP[1] = _rxMsg[5+offset+1];
            motors.at(i)->setDriveOutputPower(toUI(tmpOP));
            
            char tmpMT[2]; tmpMT[0] = _rxMsg[7+offset]; tmpMT[1] = _rxMsg[7+offset+1];
            motors.at(i)->setMotorTorque(toUI(tmpMT));
            
            char tmpMP[4];
            tmpMP[0] = _rxMsg[9+offset]; tmpMP[1] = _rxMsg[9+offset+1];
            tmpMP[2] = _rxMsg[9+offset+2]; tmpMP[3] = _rxMsg[9+offset+3];
            motors.at(i)->setMotorPosition(toSL(tmpMP));
        }
    }
    //Request Valid
    //bit 73
}
bool ofxMfsPlatform::loadConfigFile(string _file){
    bool cfgLoaded = cfg.openLocal(_file);
    if (cfgLoaded){
        
        ofLogVerbose("MOTION PLATFORM")<<"Config loaded - now loading elements";
        
        //Coms
        if (cfg.isMember("coms")){
            mbIP = cfg["coms"]["ip"].asString();
            mbPort = cfg["coms"]["port"].asInt();
            initUDPComs();
        } else {
            cfgLoaded = false;
            ofLogError("MOTION PLATFORM")<<"Load config file failed - can't find coms in json";
        }
        
        //Motion Limits
        if (cfg.isMember("motion limits")){
            pitchMin = cfg["motion limits"]["pitch"]["min"].asFloat();
            pitchMax = cfg["motion limits"]["pitch"]["max"].asFloat();
            rollMin = cfg["motion limits"]["roll"]["min"].asFloat();
            rollMax = cfg["motion limits"]["roll"]["max"].asFloat(),
            heaveMin = cfg["motion limits"]["heave"]["min"].asFloat();
            heaveMax = cfg["motion limits"]["heave"]["max"].asFloat(),
            swayMin = cfg["motion limits"]["sway"]["min"].asFloat();
            swayMax = cfg["motion limits"]["sway"]["max"].asFloat(),
            surgeMin = cfg["motion limits"]["surge"]["min"].asFloat();
            surgeMax = cfg["motion limits"]["surge"]["max"].asFloat();
            yawMin = cfg["motion limits"]["yaw"]["min"].asFloat();
            yawMax = cfg["motion limits"]["yaw"]["max"].asFloat();
        } else {
            cfgLoaded = false;
            ofLogError("MOTION PLATFORM")<<"Load config file failed - can't find motion limits in json";
        }
        
        //Platform Config
        if (cfg.isMember("platform config")){
            ofLogVerbose("MOTION PLATFORM")<<"Platform config available in JSON element";
        } else {
            cfgLoaded = false;
            ofLogError("MOTION PLATFORM")<<"Load config file failed - can't find platform config in json";
        }
        
    } else {
        ofLogError("MOTION BASE")<<"Error loading configuration";
    }
    ofLogVerbose("MOTION PLATFORM")<<"Config loaded with result: "<<cfgLoaded;
    return cfgLoaded;
}
void ofxMfsPlatform::generateConfigPacket(){
//    uint8_t localByteArray[27];
//
//    localByteArray[26] = 0x00;
//    unsigned char * t = localByteArray;
}

//Internal Functions
#pragma mark INTERNAL FUNCTIONS
void ofxMfsPlatform::updatePlatformStatus(){
    int originalState = platformModuleState;
    
    if (enableComs == false){
        platformModuleState = OFX_PLATFORM_STATE_DISABLED;
        notifyIfStateChanged(originalState);
        return;
    }
    
    //Set to disconnected
    if ((lastReceivedPacketTime+5000)<ofGetElapsedTimeMillis()){
        platformModuleState = OFX_PLATFORM_STATE_OFFLINE;
        //TODO: Clear all module as its disconnected
        notifyIfStateChanged(originalState);
        return;
    } else if (platformModuleState == OFX_PLATFORM_STATE_OFFLINE){
        platformModuleState = OFX_PLATFORM_STATE_CONNECTION_ATTEMPT;
        //TODO: Init TCP Coms
        notifyIfStateChanged(originalState);
        return;
    }
    
    //Set to Sending Config
    if (motionControllerState == 4){
        if (platformModuleState != OFX_PLATFORM_STATE_SENDING_CONFIG){
            platformModuleState = OFX_PLATFORM_STATE_SENDING_CONFIG;
            generateConfigPacket();
            //TODO: Send Config Packet;
        } else {
            platformModuleState = OFX_PLATFORM_STATE_SENDING_CONFIG;
        }
        notifyIfStateChanged(originalState);
        return;
    }
    
    //Set to standby
    if (motionControllerState == 1){
        platformModuleState = OFX_PLATFORM_STATE_STANDBY;
        notifyIfStateChanged(originalState);
        return;
    }
    
    //Set to Running
    if (motionControllerState == 0){
        platformModuleState = OFX_PLATFORM_STATE_RUNNING;
        notifyIfStateChanged(originalState);
        return;
    }
    
    //Set to Drive Disable
    if (motionControllerState == 2){
        platformModuleState = OFX_PLATFORM_STATE_DRIVE_DISABLE;
        notifyIfStateChanged(originalState);
        return;
    }
    
    //Set to fault
    if (motionControllerState == 3){
        platformModuleState = OFX_PLATFORM_STATE_FAULT;
        notifyIfStateChanged(originalState);
        return;
    }
    
}
void ofxMfsPlatform::notifyIfStateChanged(int _originalState){
    if (_originalState != platformModuleState){
        ofNotifyEvent(platformModuleStateChanged, platformModuleState, this);
    }
}


//Gets
#pragma mark GETS
float ofxMfsPlatform::getLimitPitchMin(){ return pitchMin; }
float ofxMfsPlatform::getLimitPitchMax(){ return pitchMax; }
float ofxMfsPlatform::getLimitRollMin(){ return rollMin; }
float ofxMfsPlatform::getLimitRollMax(){ return rollMax; }
float ofxMfsPlatform::getLimitHeaveMin(){ return heaveMin; }
float ofxMfsPlatform::getLimitHeaveMax(){ return heaveMax; }
float ofxMfsPlatform::getLimitSwayMin(){ return swayMin; }
float ofxMfsPlatform::getLimitSwayMax(){ return swayMax; }
float ofxMfsPlatform::getLimitSurgeMin(){ return surgeMin; }
float ofxMfsPlatform::getLimitSurgeMax(){ return surgeMax; }
float ofxMfsPlatform::getLimitYawMin(){ return yawMin; }
float ofxMfsPlatform::getLimitYawMax(){ return yawMax; }
float ofxMfsPlatform::getTargetPosPitch(){ return targetPosPitch; }
float ofxMfsPlatform::getTargetPosRoll(){ return targetPosRoll; }
float ofxMfsPlatform::getTargetPosHeave(){ return targetPosHeave; }
float ofxMfsPlatform::getTargetPosSway(){ return targetPosSway; }
float ofxMfsPlatform::getTargetPosSurge(){ return targetPosSurge; }
float ofxMfsPlatform::getTargetPosYaw(){ return targetPosYaw; }
bool ofxMfsPlatform::getIsEnabled(){ return enableComs; }
string ofxMfsPlatform::getMotorStatus(int _motor){
    if (motors.size() > _motor){
        switch (motors[_motor]->getMotorStatus()){
            case 0: { return "Not Ready"; break; }
            case 1: { return "Switch On Disabled"; break; }
            case 2: { return "Ready To Switch On"; break; }
            case 3: { return "Switched On"; break; }
            case 4: { return "Operation Enabled"; break; }
            case 5: { return "Quick Stop Active"; break; }
            case 6: { return "Fault Reaction Active";  break; }
            case 7: { return "Fault"; break; }
        }
    }
}
string ofxMfsPlatform::getPlatformStatus(){
    if (!enableComs){
        return "Coms Disabled";
    }
    
    //TODO: Add online check
    
//    switch (platformStatus){
//        case 0: { return "Platform State: Enabled"; break; }
//        case 1: { return "Platform State: Disabled"; break; }
//        case 2: { return "Platform State: Error"; break; }
//        case 3: { return "Platform State: Config Needed"; break; }
//    }
}


