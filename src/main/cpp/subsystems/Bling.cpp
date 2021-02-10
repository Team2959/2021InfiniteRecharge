
#include <subsystems/Bling.h>

Bling::Bling()
{
}

void Bling::Send(std::string message)
{
    m_serialPort.Write(message);
    m_serialPort.Flush();
}
