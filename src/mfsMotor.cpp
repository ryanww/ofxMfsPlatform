
#include "mfsMotor.h"

//Init
#pragma mark INIT
mfsMotor::mfsMotor(){
    motorLoad = 0;
    motorOverload = 0;
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
    motorLoad = (float)_val;
}
void mfsMotor::setMotorOverload(unsigned int _val){
    motorOverload = (float)_val;
}
void mfsMotor::setMotorTorque(unsigned int _val){
    motorTorque = (float)_val;
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
    driveLoad = (float)_val;
}
void mfsMotor::setDriveOverload(unsigned int _val){
    driveOverload = (float)_val;
}
void mfsMotor::setDriveMaxOverload(unsigned int _val){
    driveMaxOverload = (float)_val;
}
void mfsMotor::setDriveOutputPower(unsigned int _val){
    driveOutputPower = (int)_val;
}
void mfsMotor::setDriveOutputPowerMean(unsigned int _val){
    driveOutputPowerMean = (int)_val;
}
void mfsMotor::setDriveDcBusVoltage(unsigned int _val){
    driveDcBusVoltage = (int)_val;
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
float mfsMotor::getMotorLoad(){ return motorLoad; }
float mfsMotor::getMotorOverload(){ return motorOverload; }
int mfsMotor::getMotorTorque(){ return motorTorque; }
int mfsMotor::getMotorTemp(){ return motorTemp; }
int mfsMotor::getMotorStatus(){ return motorStatus; }
long mfsMotor::getMotorPosition(){ return motorPosition; }
//Drive
float mfsMotor::getDriveLoad(){ return driveLoad; }
float mfsMotor::getDriveOverload(){ return driveOverload; }
float mfsMotor::getDriveMaxOverload(){ return driveMaxOverload; }
int mfsMotor::getDriveOutputPower(){ return driveOutputPower; }
int mfsMotor::getDriveOutputPowerMean(){ return driveOutputPowerMean; }
int mfsMotor::getDriveDcBusVoltage(){ return driveDcBusVoltage; }
int mfsMotor::getDriveTemp(){ return driveTemp; }
int mfsMotor::getDriveLastWarning(){ return driveLastWarning; }
int mfsMotor::getDriveLastError(){ return driveLastError; }
long mfsMotor::getDriveWarningLatched(){ return driveWarningLatched; }
long mfsMotor::getDriveSigLatched(){ return driveSigLatched; }






