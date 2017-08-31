//
//  tcpCmd.c
//
//  Created by Ryan Wilkinson on 8/30/17.
//

struct tcpCmd {
public:
    tcpCmd(){
        varUsShort = 0; //type 0
        varUsInt = 0; //type 1
        varUsL = 0; //type 2
        varSL = 0; //type 3
        varType = 0;
        writeMode = false;
        index = 0x00;
        subIndex = 0x00;
    }
    void setUnsigned8(bool _writeMode, unsigned char _index, unsigned char _subIndex, unsigned short _value){
        writeMode = _writeMode;
        index = _index;
        subIndex = _subIndex;
        varType = 0;
        varUsShort = _value;
    }
    void setUnsigned16(bool _writeMode, unsigned char _index, unsigned char _subIndex, unsigned int _value){
        writeMode = _writeMode;
        index = _index;
        subIndex = _subIndex;
        varType = 1;
        varUsInt = _value;
    }
    void setUnsigned32(bool _writeMode, unsigned char _index, unsigned char _subIndex, unsigned long _value){
        writeMode = _writeMode;
        index = _index;
        subIndex = _subIndex;
        varType = 2;
        varUsL = _value;
    }
    void setSigned32(bool _writeMode, unsigned char _index, unsigned char _subIndex, signed long _value){
        writeMode = _writeMode;
        index = _index;
        subIndex = _subIndex;
        varType = 3;
        varSL = _value;
    }
    int getType(){ return varType(); }
    unsigned short getVarType0{ return varUsShort; }
    unsigned int getVarType1{ return varUsInt: }
    unsigned long getVarType2{ return varUsL; }
    signed long getVarType3{ return varSL; }
private:
    unsigned short varUsShort; //type 0
    unsigned int varUsInt; //type 1
    unsigned long varUsL; //type 2
    signed long varSL; //type 3
    int varType;
    bool writeMode;
    unsigned char index;
    unsigned char subIndex;
};
