
#include "mfsMotor.h"

//Init
#pragma mark INIT
mfsMotor::mfsMotor(){
    motorLoad = 0;
    motorOverload = 0;
    motorMaxOverload = 0;
    motorTorque = 0;
    motorTemp = 0;
    motorStatus = 0;
    motorPosition = 0;
    driveLoad = 0;
    driveOverload = 0;
    driveMaxOverload = 0;
    driveOutputPower = 0;
    driveOutputPowerMean = 0;
    driveDcBusVoltage = 0;
    driveTemp = 0;
    driveLastWarning = 0;
    driveLastError = 0;
    driveWarningLatched = 0;
    driveSigLatched = 0;
}
mfsMotor::~mfsMotor(){
    
}

//Sets
#pragma mark SETS
//Motor
void mfsMotor::setMotorLoad(unsigned int _val){
    motorLoad = (int)_val;
}
void mfsMotor::setMotorOverload(unsigned int _val){
    motorOverload = (int)_val;
}
void mfsMotor::setMotorMaxOverload(unsigned int _val){
    motorMaxOverload = (int)_val;
}
void mfsMotor::setMotorTorque(unsigned int _val){
    motorTorque = (float)_val/10;
}
void mfsMotor::setMotorTemp(unsigned int _val){
    motorTemp = (int)_val;
}
void mfsMotor::setMotorStatus(unsigned int _val){
    motorStatus = (int)_val;
}
void mfsMotor::setMotorPosition(signed long _val){
    motorPosition = (long)_val;
}

//Drive
void mfsMotor::setDriveLoad(unsigned int _val){
    driveLoad = (int)_val;
}
void mfsMotor::setDriveOverload(unsigned int _val){
    driveOverload = (int)_val;
}
void mfsMotor::setDriveMaxOverload(unsigned int _val){
    driveMaxOverload = (int)_val;
}
void mfsMotor::setDriveOutputPower(unsigned int _val){
    driveOutputPower = (int)_val;
}
void mfsMotor::setDriveOutputPowerMean(unsigned int _val){
    driveOutputPowerMean = (int)_val;
}
void mfsMotor::setDriveDcBusVoltage(unsigned int _val){
    driveDcBusVoltage = (float)_val/10;
}
void mfsMotor::setDriveTemp(unsigned int _val){
    driveTemp = (int)_val;
}
void mfsMotor::setDriveLastWarning(unsigned int _val){
    driveLastWarning = (int)_val;
}
void mfsMotor::setDriveLastError(unsigned int _val){
    driveLastError = (int)_val;
}
void mfsMotor::setDriveWarningLatched(unsigned long _val){
    driveWarningLatched = (long)_val;
}
void mfsMotor::setDriveSigLatched(unsigned long _val){
    driveSigLatched = (long)_val;
}

//Gets
#pragma mark GETS
int mfsMotor::getMotorLoad(){ return motorLoad; }
int mfsMotor::getMotorOverload(){ return motorOverload; }
float mfsMotor::getMotorTorque(){ return motorTorque; }
int mfsMotor::getMotorTemp(){ return motorTemp; }
int mfsMotor::getMotorStatus(){ return motorStatus; }
string mfsMotor::getMotorStatusText(){
    switch (motorStatus){
        case 0:{
            return "not ready to switch on";
            break;
        }
        case 1:{
            return "Switch on disabled";
            break;
        }
        case 2:{
            return "ready to switch on";
            break;
        }
        case 3:{
            return "Switched on";
            break;
        }
        case 4:{
            return "operation enabled";
            break;
        }
        case 5:{
            return "Quick stop active";
            break;
        }
        case 6:{
            return "fault reaction active";
            break;
        }
        case 7:{
            return "Fault";
            break;
        }
    }
}
float mfsMotor::getMotorPosition(){ return motorPosition; }
//Drive
int mfsMotor::getDriveLoad(){ return driveLoad; }
int mfsMotor::getDriveOverload(){ return driveOverload; }
int mfsMotor::getDriveMaxOverload(){ return driveMaxOverload; }
int mfsMotor::getDriveOutputPower(){ return driveOutputPower; }
int mfsMotor::getDriveOutputPowerMean(){ return driveOutputPowerMean; }
float mfsMotor::getDriveDcBusVoltage(){ return driveDcBusVoltage; }
int mfsMotor::getDriveTemp(){ return driveTemp; }
int mfsMotor::getDriveLastWarning(){ return driveLastWarning; }
int mfsMotor::getDriveLastError(){ return driveLastError; }
long mfsMotor::getDriveWarningLatched(){ return driveWarningLatched; }
long mfsMotor::getDriveSigLatched(){ return driveSigLatched; }






