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
    mbUdpPort = 0;
    platformModuleState = OFX_PLATFORM_STATE_OFFLINE;
    enableComs = false;
    configFileLoaded = false;
    configSentToPlatform = false;
    tcpConnected = false;
    
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
    ofLogVerbose("ofxMfsPlatform")<<"Starting Motion Base setup";
    
    //Configuration
    if (!loadConfigFile(_configFile)){
        ofLogError("ofxMfsPlatform")<<"Error loading configuration file";
        return;
    } else {
        configFileLoaded = true;
    }
    
    //TODO: Adjust for other then 6DOF platform
    for (int i=0; i<6; i++){
        motors.push_back(new mfsMotor());
    }

    initUDPComs();
    ofLogVerbose("ofxMfsPlatform")<<"Motion base setup complete - starting thread";
    startThread();
}
void ofxMfsPlatform::initUDPComs(){
    ofLogVerbose("ofxMfsPlatform")<<"Initializing UDP coms";
    //Setup UDP Receiver
    mbUdpRx.Create();
    mbUdpRx.Bind(51302);
    mbUdpRx.SetNonBlocking(true);
    
    //Setup UDP Transmitter
    ofxUDPSettings settings;
    settings.sendTo(mbIP, mbUdpPort);
    settings.blocking = false;
    mbUdpTx.Setup(settings);
    ofLogVerbose("ofxMfsPlatform")<<"Initializing UDP coms completed";
}
void ofxMfsPlatform::initTCPComs(){
    ofLogVerbose("ofxMfsPlatform")<<"Initializing TCP coms";
    ofxTCPSettings settings(mbIP, mbTcpPort);
    tcpConnected = tcp.setup(settings);
}
void ofxMfsPlatform::takePlatformOffline(){
    ofLogVerbose("ofxMfsPlatform")<<"Taking platform offline";
    configSentToPlatform = false;
    configParamCmdsToSend.clear();
    if (tcp.isConnected()){
        tcp.close();
    }
}

//Sets
#pragma mark SETS
void ofxMfsPlatform::setTargetPosition(float _pitch, float _roll, float _heave,
                                    float _sway, float _surge, float _yaw){
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
void ofxMfsPlatform::setEnabled(bool _enable){
    if (configFileLoaded == false){
        ofLogError("ofxMfsPlatform")<<"Unable to enable module - config hasn't been loaded";
        return;
    }
    if (enableComs != _enable){
        if (_enable){
            ofLogVerbose("ofxMfsPlatform")<<"Setting enable state to:"<<_enable;
            takePlatformOffline();
            enableComs = _enable;
            changePlatformStatus(OFX_PLATFORM_STATE_OFFLINE);
        } else {
            enableComs = _enable;
            ofLogVerbose("ofxMfsPlatform")<<"Setting enable state to:"<<_enable;
            changePlatformStatus(OFX_PLATFORM_STATE_DISABLED);
        }
    }
}


//Running Thread
#pragma mark RUNNING THREAD
void ofxMfsPlatform::threadedFunction(){
    setThreadName("ofxMfsPlatform");
    while(isThreadRunning()){
        
        
        //UDP RX
        tcpConnected = tcp.isConnected();
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
        if (tcpConnected){
            char tcpRxMsg[1000];
            int numBytesAvailable = tcp.getNumReceivedBytes();
            tcp.receiveRawBytes(tcpRxMsg, 1000);
            if (numBytesAvailable>0){
                cout<<"got tcp msg "<<numBytesAvailable<<"-"<<tcpRxMsg<<endl;
            }
        }
            
        //Transmit Packet
        if (platformModuleState == OFX_PLATFORM_STATE_STANDBY || platformModuleState == OFX_PLATFORM_STATE_RUNNING){
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
        
        ofLogVerbose("ofxMfsPlatform")<<"Config loaded - now loading elements";
        
        //Coms
        if (cfg.isMember("coms")){
            mbIP = cfg["coms"]["ip"].asString();
            mbUdpPort = cfg["coms"]["udp port"].asInt();
            mbTcpPort = cfg["coms"]["tcp port"].asInt();
            ofLogVerbose("ofxMfsPlatform")<<"Connection details - IP:"<<mbIP<<" Udp port:"<<mbUdpPort<<" Tcp port:"<<mbTcpPort;
        } else {
            cfgLoaded = false;
            ofLogError("ofxMfsPlatform")<<"Load config file failed - can't find coms in json";
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
            ofLogError("ofxMfsPlatform")<<"Load config file failed - can't find motion limits in json";
        }
        
        //Platform Config
        if (cfg.isMember("platform config")){
            ofLogVerbose("ofxMfsPlatform")<<"Platform config available in JSON element";
        } else {
            cfgLoaded = false;
            ofLogError("ofxMfsPlatform")<<"Load config file failed - can't find platform config in json";
        }
        
    } else {
        ofLogError("ofxMfsPlatform")<<"Error loading configuration";
    }
    ofLogVerbose("ofxMfsPlatform")<<"Config loaded with result: "<<cfgLoaded;
    return cfgLoaded;
}
void ofxMfsPlatform::generateConfigPackets(){
    ofLogVerbose("ofxMfsPlatform")<<"Generate config packets";
    
    uint8_t localByteArray[27];
    localByteArray[0] = 0x00;
    unsigned char * t = localByteArray;
    configParamCmdsToSend.push_back(t);
    
}

//Internal Functions
#pragma mark INTERNAL FUNCTIONS
void ofxMfsPlatform::updatePlatformStatus(){
    if (enableComs == false){
        changePlatformStatus(OFX_PLATFORM_STATE_DISABLED);
        return;
    }
    
    //Set to disconnected unless its seen a packet and offline
    if ((lastReceivedPacketTime+1000)<ofGetElapsedTimeMillis()){
        changePlatformStatus(OFX_PLATFORM_STATE_OFFLINE);
        return;
    } else if (platformModuleState == OFX_PLATFORM_STATE_OFFLINE){
        changePlatformStatus(OFX_PLATFORM_STATE_CONNECTION_ATTEMPT);
        return;
    }
    
    //To sending config
    if (platformModuleState == OFX_PLATFORM_STATE_CONNECTION_ATTEMPT){
        if (tcpConnected){ //Start cfg send
            if (motionControllerState == PLATFORM_INTERNAL_STATE_CONFIG_NEEDED){
                changePlatformStatus(OFX_PLATFORM_STATE_SENDING_CONFIG);
                return;
            }
        } else { //attempt more connections
            //TODO: Add more connection attempts with timeout
            return;
        }
    }
    
    //Set to standby
    if (motionControllerState == PLATFORM_INTERNAL_STATE_READY){
        changePlatformStatus(OFX_PLATFORM_STATE_STANDBY);
        return;
    }
    
    //Set to Running
    if (motionControllerState == PLATFORM_INTERNAL_STATE_ENABLED){
        changePlatformStatus(OFX_PLATFORM_STATE_RUNNING);
        return;
    }
    
    //Set to Drive Disable
    if (motionControllerState == PLATFORM_INTERNAL_STATE_DISABLED){
        changePlatformStatus(OFX_PLATFORM_STATE_DRIVE_DISABLE);
        return;
    }
    
    //Set to fault
    if (motionControllerState == PLATFORM_INTERNAL_STATE_ERROR){
        changePlatformStatus(OFX_PLATFORM_STATE_FAULT);
        return;
    }
    
}
void ofxMfsPlatform::changePlatformStatus(int _newState){
    if (_newState <= 7 && _newState >= 0){
        if (platformModuleState != _newState){
            switch (_newState){
                case OFX_PLATFORM_STATE_DISABLED: {
                    takePlatformOffline();
                    break;
                }
                case OFX_PLATFORM_STATE_OFFLINE: {
                    break;
                }
                case OFX_PLATFORM_STATE_CONNECTION_ATTEMPT: {
                    initTCPComs();
                    break;
                }
                case OFX_PLATFORM_STATE_SENDING_CONFIG: {
                    generateConfigPackets();
                    break;
                }
                case OFX_PLATFORM_STATE_STANDBY: {
                    
                    break;
                }
                case OFX_PLATFORM_STATE_RUNNING: {
                    
                    break;
                }
                case OFX_PLATFORM_STATE_DRIVE_DISABLE: {
                    
                    break;
                }
                case OFX_PLATFORM_STATE_FAULT: {
                    
                    break;
                }
            }
            platformModuleState = _newState;
            ofLogVerbose("ofxMfsPlatform")<<"State changed to: "<<getPlatformModuleStateAsString();
            ofNotifyEvent(platformModuleStateChanged, platformModuleState, this);
        }
        platformModuleState = _newState;
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
int ofxMfsPlatform::getPlatformModuleState(){
    return platformModuleState;
}
string ofxMfsPlatform::getPlatformModuleStateAsString(){
    switch (platformModuleState){
        case OFX_PLATFORM_STATE_DISABLED:{
            return "Disabled";
            break;
        }
        case OFX_PLATFORM_STATE_OFFLINE:{
            return "Offline";
            break;
        }
        case OFX_PLATFORM_STATE_CONNECTION_ATTEMPT:{
            return "Attempting Connection";
            break;
        }
        case OFX_PLATFORM_STATE_SENDING_CONFIG:{
            return "Sending Config";
            break;
        }
        case OFX_PLATFORM_STATE_STANDBY:{
            return "Standby";
            break;
        }
        case OFX_PLATFORM_STATE_RUNNING:{
            return "Running";
            break;
        }
        case OFX_PLATFORM_STATE_DRIVE_DISABLE:{
            return "Drive Disabled";
            break;
        }
        case OFX_PLATFORM_STATE_FAULT:{
            return "Fault";
            break;
        }
    }
}


