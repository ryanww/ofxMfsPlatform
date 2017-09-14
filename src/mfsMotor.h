//  Created by Ryan Wilkinson on 7/22/17.
//

#pragma once

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
    float getMotorLoad();
    float getMotorOverload();
    int getMotorTorque();
    int getMotorTemp();
    int getMotorStatus();
    long getMotorPosition();
    //Drive
    float getDriveLoad();
    float getDriveOverload();
    float getDriveMaxOverload();
    int getDriveOutputPower();
    int getDriveOutputPowerMean();
    int getDriveDcBusVoltage();
    int getDriveTemp();
    int getDriveLastWarning();
    int getDriveLastError();
    long getDriveWarningLatched();
    long getDriveSigLatched();
    
private:
    
    //Vars
    //Motor
    float motorLoad; //%
    float motorOverload; //%
    float motorMaxOverload; //%
    int motorTorque; //Nm
    int motorTemp; //°c
    int motorStatus;
    long motorPosition;
    
    //Drive
    float driveLoad; //%
    float driveOverload; //%
    float driveMaxOverload; //%
    int driveOutputPower; //watts
    int driveOutputPowerMean; //watts
    int driveDcBusVoltage; //vdc
    int driveTemp; //°c
    int driveLastWarning;
    int driveLastError;
    long driveWarningLatched;
    long driveSigLatched;
    
    
};

