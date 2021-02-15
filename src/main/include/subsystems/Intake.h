#pragma once

#include <frc/DigitalInput.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <utility/StickySwitch.h>
#include <RobotMap.h>
#include <frc/DoubleSolenoid.h>

class Intake
{
private:
    cwtech::StickySwitch m_newPowercellSensor {kNewPowercellSensor};
    cwtech::StickySwitch m_securedPowercellSensor {kSecuredPowercellSensor};
    cwtech::StickySwitch m_kickerSensor {kKickerSensor};

    cwtech::StickySwitch m_leftFeederSensor{kIntakeLeftFeederSensor};
    cwtech::StickySwitch m_rightFeederSensor{kIntakeRightFeederSensor};

    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_intake {kIntakeVictorSpxCanId};
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_kicker {kKickerVictorSpxCanId};
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_conveyor {kConveyorVictorSpxCanId};
    
    frc::DoubleSolenoid m_rightFeeder {2,3};
    frc::DoubleSolenoid m_leftFeeder {4,5};

    // Smart Dashboard
    const std::string kIntakeName = "Intake/";
    const std::string kConveyorName = "Conveyor/";
    const std::string kKickerName = "Kicker/";
    const std::string kDebug = kIntakeName + "Debug";
    const std::string kConveyorSpeed = kConveyorName + "Speed";
    const std::string kKickerSpeed = kKickerName + "Speed";
    const std::string kIntakeSpeed = kIntakeName + "Speed";
    const std::string kIntakeState = kIntakeName + "State";
    const std::string kConveyorSpeedWhenLoading = kConveyorName + "Speed When Loading";

    bool m_debugEnable = false;

    const double kFullIntakeSpeed = 0.75;
    const double kFullConveyorSpeed = 0.6;
    const double kFullKickerSpeed = 0.3;
    const double kFullConveyorSpeedWhenLoading = 1.0;
    double m_intakeSpeed = kFullIntakeSpeed;
    double m_conveyorSpeed = kFullConveyorSpeed;
    double m_conveyorSpeedWhenLoading = kFullConveyorSpeedWhenLoading;
    double m_kickerSpeed = kFullKickerSpeed;

    std::string GetIntakeStateText();

public:
    enum class SensorLocation
    {
        Kicker,
        NewPowercell,
        SecuredPowercell
    };

    void OnRobotInit();
    void OnRobotPeriodic();
    void ProcessStickySwitches();

    double GetIntakeFullSpeed() const;
    double GetConveyorFullSpeed() const;
    double GetConveyorFullSpeedWhenLoading() const;
    double GetKickerFullSpeed() const;
    bool IsIntakeRunning() const;

    void SetIntakeSpeed(double speed);
    void SetConveyorSpeed(double speed);
    void SetKickerSpeed(double speed);

    enum FeedingState
    {
        Neither,
        Left,
        Right,
    };
    FeedingState m_feedingState = FeedingState::Neither;
    int m_feedingSteps = 0;
    void Feed();

    bool GetSensor(SensorLocation location);
    bool GetSensorPressed(SensorLocation location);
    bool GetSensorReleased(SensorLocation location);

    void LeftBallFlipper(bool state);
    void RightBallFlipper(bool state);
    bool GetLeftBallFlipper();
    bool GetRightBallFlipper();
    bool GetLeftBallFlipperSensor();
    bool GetRightBallFlipperSensor();
};