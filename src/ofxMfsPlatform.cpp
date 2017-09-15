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
    posTxWait = 10;
    lastPosPacketTxTime = 0;
    tcpTxWait = 10;
    lastTcpPacketTxTime = 0;
    
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
    motionControllerState = 2;
    uptimeCounter = 0;
    lastReceivedPacketTime = 0;
}
ofxMfsPlatform::~ofxMfsPlatform(){
    
    //Send to Standby
    uint8_t stbCmd[7];
    stbCmd[0] = 0x10;
    stbCmd[1] = 0x02;
    stbCmd[2] = 0x03;
    stbCmd[3] = 0x02;
    stbCmd[4] = 0x00;
    stbCmd[5] = 0x00;
    stbCmd[6] = 0x17;
    if (tcp.isConnected()){
        tcp.sendRawBytes((const char*)&stbCmd, sizeof(stbCmd));
    }
    
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
    tcpCmdsToSend.clear();
    if (tcp.isConnected()){
        tcp.close();
    }
}

//Sets
#pragma mark SETS
void ofxMfsPlatform::setTargetPosition(float _pitch, float _roll, float _heave,
                                    float _sway, float _surge, float _yaw){
    targetPosPitch = CLAMP(_pitch, pitchMin, pitchMax);
    targetPosPitchInt = (signed int)(targetPosPitch*100);
    targetPosRoll = CLAMP(_roll, rollMin, rollMax);
    targetPosRollInt = (signed int)(targetPosRoll*100);
    targetPosHeave = CLAMP(_heave, heaveMin, heaveMax);
    targetPosHeaveInt = (signed int)(targetPosHeave*100);
    targetPosSway = CLAMP(_sway, swayMin, swayMax);
    targetPosSwayInt = (signed int)(targetPosSway*100);
    targetPosSurge = CLAMP(_surge, surgeMin, surgeMax);
    targetPosSurgeInt = (signed int)(targetPosSurge*100);
    targetPosYaw = CLAMP(_yaw, yawMin, yawMax);
    targetPosYawInt = (signed int)(targetPosYaw*100);
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
void ofxMfsPlatform::resetErrors(){
    if (platformModuleState == OFX_PLATFORM_STATE_DRIVE_DISABLE || platformModuleState == OFX_PLATFORM_STATE_FAULT){
        ofLogVerbose("ofxMfsPlatform")<<"Sending reset command";
        bitset<16> modeWord;
        modeWord[5] = 1; //Clear Errors
        modeWord[4] = 0; //Calibrate
        modeWord[3] = 0; //Mode 2
        modeWord[2] = 0; //Mode 1
        modeWord[1] = 0; //Mode 0
        
        tcpCmd *rst = new tcpCmd();
        rst->setUnsigned16(true, 0x02, 0x01, modeWord.to_ullong());
        tcpCmdsToSend.push_back(rst);
    } else {
        ofLogError("ofxMfsPlatform")<<"Error sending reset - platform not in correct state";
    }
}
void ofxMfsPlatform::setMotionState(bool _enable){
    if (platformModuleState > OFX_PLATFORM_STATE_SENDING_CONFIG){
        if (_enable){
            if (platformModuleState == OFX_PLATFORM_STATE_STANDBY){
                ofLogVerbose("ofxMfsPlatform")<<"Sending to run mode";
                
                //Multi-axis mode
                bitset<16> modeWord;
                modeWord[5] = 0; //Clear Errors
                modeWord[4] = 0; //Calibrate
                modeWord[3] = 0; //Mode 2
                modeWord[2] = 0; //Mode 1
                modeWord[1] = 0; //Mode 0
                tcpCmd *maMode = new tcpCmd();
                maMode->setUnsigned16(true, 0x02, 0x01, modeWord.to_ullong());
                tcpCmdsToSend.push_back(maMode);
                
                //Enable Mode
                bitset<16> enableControlWord;
                enableControlWord[15] = 0; //Motor 6
                enableControlWord[14] = 0; //Motor 5
                enableControlWord[13] = 0; //Motor 4
                enableControlWord[12] = 0; //Motor 3
                enableControlWord[11] = 0; //Motor 2
                enableControlWord[10] = 0; //Motor 1
                enableControlWord[0] = 1; //Enable
                tcpCmd *en = new tcpCmd();
                en->setUnsigned16(true, 0x02, 0x02, enableControlWord.to_ullong());
                tcpCmdsToSend.push_back(en);
            } else {
                ofLogError("ofxMfsPlatform")<<"Error sending reset - platform not in correct state";
            }
        } else {
            if (platformModuleState == OFX_PLATFORM_STATE_RUNNING ||
                platformModuleState == OFX_PLATFORM_STATE_DRIVE_DISABLE ||
                platformModuleState == OFX_PLATFORM_STATE_FAULT){
                bitset<16> disableControlWord;
                disableControlWord[15] = 0; //Motor 6
                disableControlWord[14] = 0; //Motor 5
                disableControlWord[13] = 0; //Motor 4
                disableControlWord[12] = 0; //Motor 3
                disableControlWord[11] = 0; //Motor 2
                disableControlWord[10] = 0; //Motor 1
                disableControlWord[0] = 1; //Disable
                tcpCmd *pfDisable = new tcpCmd();
                ofLogVerbose("ofxMfsPlatform")<<"Sending to disable mode";
                pfDisable->setUnsigned16(true, 0x02, 0x03, disableControlWord.to_ullong());
                tcpCmdsToSend.push_back(pfDisable);
            } else {
                ofLogError("ofxMfsPlatform")<<"Error sending reset - platform not in correct state";
            }
        }
    } else {
        ofLogError("ofxMfsPlatform")<<"Error sending reset - platform not in correct state";
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
                lastReceivedPacketTime = ofGetElapsedTimeMillis();
            }
            
            //Send
            if (tcpCmdsToSend.size()>0 && ofGetElapsedTimeMillis()> (lastTcpPacketTxTime+tcpTxWait)){
                if (tcpCmdsToSend[0]->getType() == 0){ //Unsigned Short
                    uint8_t dataArray[1];
                    int arrayLength = 5+sizeof(dataArray);
                    uint8_t toSend[arrayLength];
                    dataArray[0] = LOWBYTE(tcpCmdsToSend[0]->getVarType0());
                    
                    //Control word
                    bitset<8> controlWord;
                    controlWord.reset();
                    controlWord[7] = !tcpCmdsToSend[0]->getWriteMode();
                    controlWord[6] = 0;
                    controlWord[5] = 0;
                    controlWord[4] = 1;
                    toSend[0] = controlWord.to_ullong();
                    
                    //Index
                    toSend[1] = tcpCmdsToSend[0]->getIndex();
                    //Sub Index
                    toSend[2] = tcpCmdsToSend[0]->getSubIndex();
                    //length
                    toSend[3] = sizeof(dataArray);
                    
                    //Data
                    for (int i=0; i<sizeof(dataArray); i++){
                        toSend[4+i] = dataArray[i];
                    }
                    
                    //CRC
                    toSend[sizeof(toSend)-1] = 0;
                    unsigned char crcTmpVal = 0x00;
                    for (int i=0; i<sizeof(toSend); i++){
                        crcTmpVal = crcTmpVal + toSend[i];
                    }
                    toSend[arrayLength-1] = crcTmpVal & 0xff;
                    
                    tcp.sendRawBytes((const char*)&toSend, sizeof(toSend));
                    delete tcpCmdsToSend[0];
                    tcpCmdsToSend.erase(tcpCmdsToSend.begin());
                    lastTcpPacketTxTime = ofGetElapsedTimeMillis();
                }
                if (tcpCmdsToSend[0]->getType() == 1){ //Unsigned Int
                    uint8_t dataArray[2];
                    int arrayLength = 5+sizeof(dataArray);
                    uint8_t toSend[arrayLength];
                    dataArray[0] = HIGHBYTE(tcpCmdsToSend[0]->getVarType1());
                    dataArray[1] = LOWBYTE(tcpCmdsToSend[0]->getVarType1());
                    
                    //Control word
                    bitset<8> controlWord;
                    controlWord[7] = !tcpCmdsToSend[0]->getWriteMode();
                    controlWord[6] = 0;
                    controlWord[5] = 0;
                    controlWord[4] = 1;
                    toSend[0] = controlWord.to_ullong();
                    
                    //Index
                    toSend[1] = tcpCmdsToSend[0]->getIndex();
                    //Sub Index
                    toSend[2] = tcpCmdsToSend[0]->getSubIndex();
                    //length
                    toSend[3] = sizeof(dataArray);
                    
                    //Data
                    for (int i=0; i<sizeof(dataArray); i++){
                        toSend[4+i] = dataArray[i];
                    }
                    
                    //CRC
                    toSend[sizeof(toSend)-1] = 0;
                    unsigned char crcTmpVal = 0x00;
                    for (int i=0; i<sizeof(toSend); i++){
                        crcTmpVal = crcTmpVal + toSend[i];
                    }
                    toSend[arrayLength-1] = crcTmpVal & 0xff;
                    
                    tcp.sendRawBytes((const char*)&toSend, sizeof(toSend));
                    delete tcpCmdsToSend[0];
                    tcpCmdsToSend.erase(tcpCmdsToSend.begin());
                    lastTcpPacketTxTime = ofGetElapsedTimeMillis();
                }
            }
        } else if (platformModuleState > OFX_PLATFORM_STATE_CONNECTION_ATTEMPT){
            changePlatformStatus(OFX_PLATFORM_STATE_OFFLINE);
        }
            
        //Transmit Packet
        if (platformModuleState == OFX_PLATFORM_STATE_RUNNING){
            if(ofGetElapsedTimeMillis()>(lastPosPacketTxTime+posTxWait)){
                lastPosPacketTxTime = ofGetElapsedTimeMillis();
                uint8_t localByteArray[27];
                localByteArray[0] = 0x07;
                localByteArray[1] = 0x18;
                
                uint8_t pitchArray[4];
                memcpy(pitchArray, (unsigned char *)&targetPosPitchInt, sizeof(targetPosPitchInt));
                localByteArray[2] = pitchArray[3];
                localByteArray[3] = pitchArray[2];
                localByteArray[4] = pitchArray[1];
                localByteArray[5] = pitchArray[0];
                
                uint8_t rollArray[4];
                memcpy(rollArray, (unsigned char *)&targetPosRollInt, sizeof(targetPosRollInt));
                localByteArray[6] = rollArray[3];
                localByteArray[7] = rollArray[2];
                localByteArray[8] = rollArray[1];
                localByteArray[9] = rollArray[0];

                uint8_t heaveArray[4];
                memcpy(heaveArray, (unsigned char *)&targetPosHeaveInt, sizeof(targetPosHeaveInt));
                localByteArray[10] = heaveArray[3];
                localByteArray[11] = heaveArray[2];
                localByteArray[12] = heaveArray[1];
                localByteArray[13] = heaveArray[0];

                uint8_t swayArray[4];
                memcpy(swayArray, (unsigned char *)&targetPosSwayInt, sizeof(targetPosSwayInt));
                localByteArray[14] = swayArray[3];
                localByteArray[15] = swayArray[2];
                localByteArray[16] = swayArray[1];
                localByteArray[17] = swayArray[0];

                uint8_t surgeArray[4];
                memcpy(surgeArray, (unsigned char *)&targetPosSurgeInt, sizeof(targetPosSurgeInt));
                localByteArray[18] = surgeArray[3];
                localByteArray[19] = surgeArray[2];
                localByteArray[20] = surgeArray[1];
                localByteArray[21] = surgeArray[0];

                uint8_t yawArray[4];
                memcpy(yawArray, (unsigned char *)&targetPosYawInt, sizeof(targetPosYawInt));
                localByteArray[22] = yawArray[3];
                localByteArray[23] = yawArray[2];
                localByteArray[24] = yawArray[1];
                localByteArray[25] = yawArray[0];
                
                localByteArray[26] = 0x00;
                unsigned char * t = localByteArray;
                mbUdpTx.Send((const char*)t, sizeof(localByteArray));
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
            motors.at(i)->setMotorMaxOverload(toUI(tmpMOl));
            
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
    
    //Set config mode
    tcpCmd *entCfg = new tcpCmd();
    entCfg->setUnsigned16(true, 0x05, 0x01, 1);
    ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Enter Config Mode";
    tcpCmdsToSend.push_back(entCfg);
    
    int numMotors = 6;
    unsigned char startIndex = 0x03;
    //Set motor gear ratio and ppr
    for (int i = 0; i<numMotors; i++){
        //Motor number
        int motNum;
        if (i == 0){ motNum = 1; }
        if (i == 1){ motNum = 4; }
        if (i == 2){ motNum = 2; }
        if (i == 3){ motNum = 5; }
        if (i == 4){ motNum = 3; }
        if (i == 5){ motNum = 6; }
        tcpCmd *motNumpk = new tcpCmd();
        motNumpk->setUnsigned8(true, 0x05, startIndex, motNum);
        tcpCmdsToSend.push_back(motNumpk);
        ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set motor number: "<<motNum;
        startIndex += 0x01;
        
        //Motor gear ratio
        tcpCmd *mGr = new tcpCmd();
        mGr->setUnsigned16(true, 0x05, startIndex, cfg["platform config"]["motor "+ofToString(i+1)]["gear ratio"].asUInt());
        ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set motor "<<i+1<<" gear ratio: "<<cfg["platform config"]["motor "+ofToString(i+1)]["gear ratio"].asUInt();
        tcpCmdsToSend.push_back(mGr);
        startIndex += 0x01;
        
        //Prep PPR
        tcpCmd *mPrepPPR = new tcpCmd();
        mPrepPPR->setUnsigned8(true, 0x05, startIndex, 1);
        ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Prep next command "<<1;
        tcpCmdsToSend.push_back(mPrepPPR);
        startIndex += 0x01;
        
        //Motor PPR
        tcpCmd *mPpr = new tcpCmd();
        mPpr->setUnsigned16(true, 0x05, startIndex, cfg["platform config"]["motor "+ofToString(i+1)]["pulses per round"].asUInt());
        ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set motor "<<i+1<<" ppr: "<<cfg["platform config"]["motor "+ofToString(i+1)]["pulses per round"].asUInt();
        tcpCmdsToSend.push_back(mPpr);
        startIndex += 0x01;
    }
    
    //Platform type
    tcpCmd *pltType = new tcpCmd();
    pltType->setUnsigned16(true, 0x05, 0x02, cfg["platform config"]["platform type"].asUInt());
    ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set platform type: "<<cfg["platform config"]["platform type"].asUInt();
    tcpCmdsToSend.push_back(pltType);
    
    //Release config mode
    tcpCmd *relCfg = new tcpCmd();
    ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Exit config mode";
    relCfg->setUnsigned16(true, 0x05, 0x01, 0);
    tcpCmdsToSend.push_back(relCfg);
    
    //Platform Sensitivity
    tcpCmd *pfSnsty = new tcpCmd();
    pfSnsty->setUnsigned16(true, 0x06, 0x0d, cfg["platform config"]["sensitivity percent"].asUInt());
    ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set platform sensitivity to: "<<cfg["platform config"]["sensitivity percent"].asUInt()<<"%";
    tcpCmdsToSend.push_back(pfSnsty);
    
    //Platform Acceleration
    tcpCmd *pfAcel = new tcpCmd();
    pfAcel->setUnsigned16(true, 0x06, 0x0f, cfg["platform config"]["acceleration percent"].asUInt());
    ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set platform acceleration to: "<<cfg["platform config"]["acceleration percent"].asUInt()<<"%";
    tcpCmdsToSend.push_back(pfAcel);
    
    //Platform Deceleration
    tcpCmd *pfDecel = new tcpCmd();
    pfDecel->setUnsigned16(true, 0x06, 0x10, cfg["platform config"]["deceleration percent"].asUInt());
    ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set platform deceleration to: "<<cfg["platform config"]["deceleration percent"].asUInt()<<"%";
    tcpCmdsToSend.push_back(pfDecel);
    
    //Jerk Limitation
    tcpCmd *pfJkLim = new tcpCmd();
    pfJkLim->setUnsigned16(true, 0x06, 0x11, cfg["platform config"]["jerk limitation"].asUInt());
    ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set jerk limitation to: "<<cfg["platform config"]["jerk limitation"].asUInt();
    tcpCmdsToSend.push_back(pfJkLim);
    
    //Set Limitations
    startIndex = 0x01;
    for (int i = 0; i<numMotors; i++){
        //Max limit in %
        tcpCmd *maxLimit = new tcpCmd();
        maxLimit->setUnsigned16(true, 0x06, startIndex, cfg["platform config"]["motor "+ofToString(i+1)]["max percent position"].asUInt());
        ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set max limit for motor "<<i+1<<" to: "<<cfg["platform config"]["motor "+ofToString(i+1)]["max percent position"].asUInt()<<"%";
        tcpCmdsToSend.push_back(maxLimit);
        startIndex += 0x01;
        
        //Min limit in %
        tcpCmd *minLimit = new tcpCmd();
        minLimit->setUnsigned16(true, 0x06, startIndex, cfg["platform config"]["motor "+ofToString(i+1)]["min percent position"].asUInt());
        ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set min limit for motor "<<i+1<<" to: "<<cfg["platform config"]["motor "+ofToString(i+1)]["min percent position"].asUInt()<<"%";
        tcpCmdsToSend.push_back(minLimit);
        startIndex += 0x01;
    }

    
    //CTRL1_KPp
    tcpCmd *pfCTRL1_KPp = new tcpCmd();
    pfCTRL1_KPp->setUnsigned16(true, 0x0D, 0x02, cfg["platform config"]["multi-axis position mode"]["CTRL1_KPp"].asUInt());
    ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set multi-axis parameter CTRL1_KPp to:"<<cfg["platform config"]["multi-axis position mode"]["CTRL1_KPp"].asUInt();
    tcpCmdsToSend.push_back(pfCTRL1_KPp);
    
    //CTRL1_KFPp
    tcpCmd *pfCTRL1_KFPp = new tcpCmd();
    pfCTRL1_KFPp->setUnsigned16(true, 0x0D, 0x03, cfg["platform config"]["multi-axis position mode"]["CTRL1_KFPp"].asUInt());
    ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set multi-axis parameter CTRL1_KFPp to:"<<cfg["platform config"]["multi-axis position mode"]["CTRL1_KFPp"].asUInt();
    tcpCmdsToSend.push_back(pfCTRL1_KFPp);
    
    //CTRL1_KPn
    tcpCmd *pfCTRL1_KPn = new tcpCmd();
    pfCTRL1_KPn->setUnsigned16(true, 0x0D, 0x04, cfg["platform config"]["multi-axis position mode"]["CTRL1_KPn"].asUInt());
    ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set multi-axis parameter CTRL1_KPn to:"<<cfg["platform config"]["multi-axis position mode"]["CTRL1_KPn"].asUInt();
    tcpCmdsToSend.push_back(pfCTRL1_KPn);
    
    //CTRL1_Tn
    tcpCmd *pfCTRL1_Tn = new tcpCmd();
    pfCTRL1_Tn->setUnsigned16(true, 0x0D, 0x05, cfg["platform config"]["multi-axis position mode"]["CTRL1_Tn"].asUInt());
    ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set multi-axis parameter CTRL1_Tn to:"<<cfg["platform config"]["multi-axis position mode"]["CTRL1_Tn"].asUInt();
    tcpCmdsToSend.push_back(pfCTRL1_Tn);
    
    //CTRL1_Nf1bandw
    tcpCmd *pfCTRL1_Nf1bandw = new tcpCmd();
    pfCTRL1_Nf1bandw->setUnsigned16(true, 0x0D, 0x0a, cfg["platform config"]["multi-axis position mode"]["CTRL1_Nf1bandw"].asUInt());
    ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set multi-axis parameter CTRL1_Nf1bandw to:"<<cfg["platform config"]["multi-axis position mode"]["CTRL1_Nf1bandw"].asUInt();
    tcpCmdsToSend.push_back(pfCTRL1_Nf1bandw);
    
    //CTRL1_KFAcc
    tcpCmd *pfCTRL1_KFAcc = new tcpCmd();
    pfCTRL1_KFAcc->setUnsigned16(true, 0x0D, 0x0c, cfg["platform config"]["multi-axis position mode"]["CTRL1_KFAcc"].asUInt());
    ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set multi-axis parameter CTRL1_KFAcc to:"<<cfg["platform config"]["multi-axis position mode"]["CTRL1_KFAcc"].asUInt();
    tcpCmdsToSend.push_back(pfCTRL1_KFAcc);
    
    //maybe jerk in here again
    
    //CTRL1_TAUiref
    tcpCmd *pfCTRL1_TAUiref = new tcpCmd();
    pfCTRL1_TAUiref->setUnsigned16(true, 0x0D, 0x06, cfg["platform config"]["multi-axis position mode"]["CTRL1_TAUiref"].asUInt());
    ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set multi-axis parameter CTRL1_TAUiref to:"<<cfg["platform config"]["multi-axis position mode"]["CTRL1_TAUiref"].asUInt();
    tcpCmdsToSend.push_back(pfCTRL1_TAUiref);
    
    //CTRL1_TAUnref
    tcpCmd *pfCTRL1_TAUnref = new tcpCmd();
    pfCTRL1_TAUnref->setUnsigned16(true, 0x0D, 0x07, cfg["platform config"]["multi-axis position mode"]["CTRL1_TAUnref"].asUInt());
    ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set multi-axis parameter CTRL1_TAUnref to:"<<cfg["platform config"]["multi-axis position mode"]["CTRL1_TAUnref"].asUInt();
    tcpCmdsToSend.push_back(pfCTRL1_TAUnref);
    
    //CTRL1_Nf1freq
    tcpCmd *pfCTRL1_Nf1freq = new tcpCmd();
    pfCTRL1_Nf1freq->setUnsigned16(true, 0x0D, 0x08, cfg["platform config"]["multi-axis position mode"]["CTRL1_Nf1freq"].asUInt());
    ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set multi-axis parameter CTRL1_Nf1freq to:"<<cfg["platform config"]["multi-axis position mode"]["CTRL1_Nf1freq"].asUInt();
    tcpCmdsToSend.push_back(pfCTRL1_Nf1freq);
    
    //CTRL1_Nf1damp
    tcpCmd *pfCTRL1_Nf1damp = new tcpCmd();
    pfCTRL1_Nf1damp->setUnsigned16(true, 0x0D, 0x09, cfg["platform config"]["multi-axis position mode"]["CTRL1_Nf1damp"].asUInt());
    ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set multi-axis parameter CTRL1_Nf1damp to:"<<cfg["platform config"]["multi-axis position mode"]["CTRL1_Nf1damp"].asUInt();
    tcpCmdsToSend.push_back(pfCTRL1_Nf1damp);
    
    //CTRL1_Kfric
    tcpCmd *pfCTRL1_Kfric = new tcpCmd();
    pfCTRL1_Kfric->setUnsigned16(true, 0x0D, 0x0b, cfg["platform config"]["multi-axis position mode"]["CTRL1_Kfric"].asUInt());
    ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set multi-axis parameter CTRL1_Kfric to:"<<cfg["platform config"]["multi-axis position mode"]["CTRL1_Kfric"].asUInt();
    tcpCmdsToSend.push_back(pfCTRL1_Kfric);
    
    //CTRL_IMAX
    tcpCmd *pfCTRL_IMAX = new tcpCmd();
    pfCTRL_IMAX->setUnsigned16(true, 0x0D, 0x0d, cfg["platform config"]["multi-axis position mode"]["CTRL_IMAX"].asUInt());
    ofLogVerbose("ofxMfsPlatform")<<"Add TCP Packet - Set multi-axis parameter CTRL_IMAX to:"<<cfg["platform config"]["multi-axis position mode"]["CTRL_IMAX"].asUInt();
    tcpCmdsToSend.push_back(pfCTRL_IMAX);
    
    //To Disable Mode
    bitset<16> disableControlWord;
    disableControlWord[15] = 0; //Motor 6
    disableControlWord[14] = 0; //Motor 5
    disableControlWord[13] = 0; //Motor 4
    disableControlWord[12] = 0; //Motor 3
    disableControlWord[11] = 0; //Motor 2
    disableControlWord[10] = 0; //Motor 1
    disableControlWord[0] = 1; //Disable
    tcpCmd *pfDisable = new tcpCmd();
    ofLogVerbose("ofxMfsPlatform")<<"Sending to disable mode";
    pfDisable->setUnsigned16(true, 0x02, 0x03, disableControlWord.to_ullong());
    tcpCmdsToSend.push_back(pfDisable);
    
    
    ofLogVerbose("ofxMfsPlatform")<<"Generate config packets completed";
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
bool ofxMfsPlatform::getIsStandby(){ return platformModuleState == OFX_PLATFORM_STATE_STANDBY; }
bool ofxMfsPlatform::getIsRunning(){ return platformModuleState == OFX_PLATFORM_STATE_RUNNING; }
bool ofxMfsPlatform::getIsFault(){ return platformModuleState == OFX_PLATFORM_STATE_FAULT; }
bool ofxMfsPlatform::getIsDriveDisabled(){ return platformModuleState == OFX_PLATFORM_STATE_DRIVE_DISABLE; }
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
mfsMotor * ofxMfsPlatform::getMotor(int _motor){
    if (_motor < motors.size()){
        return motors[_motor];
    } else {
        return NULL;
    }
}
long ofxMfsPlatform::getUptime(){
    return (long)uptimeCounter;
}






