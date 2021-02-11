
#pragma once

#include <frc/SerialPort.h>

class Bling
{
private:
    frc::SerialPort m_serialPort{9600, frc::SerialPort::kOnboard};
public:
    enum BlingState 
    {
        Unknown,
        TurningLeft,
        TurningRight,
        Braking
    };
    Bling();
    void Send(std::string message);
    void SetState(BlingState state);
    BlingState GetState();
private:
    void SendRaw(std::string message);    
    BlingState m_state;
};
