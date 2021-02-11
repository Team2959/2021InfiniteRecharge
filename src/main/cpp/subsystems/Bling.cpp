
#include <subsystems/Bling.h>

Bling::Bling()
{
}

void Bling::Send(std::string message)
{
    m_state = BlingState::Unknown;
    SendRaw(message);
}

void Bling::SendRaw(std::string message)
{
    m_serialPort.Write(message);
    m_serialPort.Flush();
}

void Bling::SetState(BlingState state)
{
    m_state = state;
    if(m_state == BlingState::TurningLeft)
    {
        Send("LEFT");
    }
    else if(m_state == BlingState::TurningRight)
    {
        Send("RIGHT");
    }
    else if(m_state == BlingState::Braking)
    {
        Send(""); // TODO look at api and figure out what to use here
    }
}
