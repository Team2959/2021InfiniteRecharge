
#pragma once

#include <frc/SerialPort.h>

class Bling
{
private:
    frc::SerialPort m_serialPort{9600, frc::SerialPort::kOnboard};
public:
    Bling();
    void Send(std::string message);
};
