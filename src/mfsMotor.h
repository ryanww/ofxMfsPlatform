//  Created by Ryan Wilkinson on 7/22/17.
//

#pragma once
#include "ofMain.h"

class mfsMotor {
    
public:
    //Init
    mfsMotor();
    ~mfsMotor();
    
    //Sets
    //Motor
    void setMotorLoad(unsigned int _val);
    void setMotorOverload(unsigned int _val);
    void setMotorMaxOverload(unsigned int _val);
    void setMotorTorque(unsigned int _val);
    void setMotorTemp(unsigned int _val);
    void setMotorStatus(unsigned int _val);
    void setMotorPosition(signed long _val);
    
    //Drive
    void setDriveLoad(unsigned int _val);
    void setDriveOverload(unsigned int _val);
    void setDriveMaxOverload(unsigned int _val);
    void setDriveOutputPower(unsigned int _val);
    void setDriveOutputPowerMean(unsigned int _val);
    void setDriveDcBusVoltage(unsigned int _val);
    void setDriveTemp(unsigned int _val);
    void setDriveLastWarning(unsigned int _val);
    void setDriveLastError(unsigned int _val);
    void setDriveWarningLatched(unsigned long _val);
    void setDriveSigLatched(unsigned long _val);
    
    
    //Gets
    //Motor
    int getMotorLoad();
    int getMotorOverload();
    float getMotorTorque();
    int getMotorTemp();
    int getMotorStatus();
    string getMotorStatusText();
    float getMotorPosition();
    //Drive
    int getDriveLoad();
    int getDriveOverload();
    int getDriveMaxOverload();
    int getDriveOutputPower();
    int getDriveOutputPowerMean();
    float getDriveDcBusVoltage();
    int getDriveTemp();
    int getDriveLastWarning();
    int getDriveLastError();
    long getDriveWarningLatched();
    long getDriveSigLatched();
    
private:
    
    //Vars
    //Motor
    int motorLoad; //%
    int motorOverload; //%
    int motorMaxOverload; //%
    float motorTorque; //0.1%
    int motorTemp;  //°c
    int motorStatus;
    float motorPosition; //0.01°
    
    //Drive
    int driveLoad; //%
    int driveOverload; //%
    int driveMaxOverload; //%
    int driveOutputPower; //watts
    int driveOutputPowerMean; //watts
    float driveDcBusVoltage; //0.1vdc
    int driveTemp; //°c
    int driveLastWarning;
    int driveLastError;
    long driveWarningLatched;
    long driveSigLatched;
    
    
};

