/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

const char* DriverCameraMode = "Driver Camera Mode";

void Robot::RobotInit() 
{
    m_drivetrain.InitalShowToSmartDashboard();
    m_intake.OnRobotInit();
    m_shooter.OnRobotInit();
    // m_colorWheel.OnRobotInit();
    m_climb.OnRobotInit();
    m_vision.OnRopotInit();
    m_autonomous.OnRobotInit();
    m_stateManager.OnRobotInit();

    m_driverSpeedConditioning.SetDeadband(kDefaultDeadband);
    m_driverSpeedConditioning.SetRange(kDefaultOutputOffset, 1.0);
    m_driverSpeedConditioning.SetExponent(kDefaultExponent);

    m_driverRotationConditioning.SetDeadband(kDefaultDeadband);
    m_driverRotationConditioning.SetRange(kDefaultOutputOffset, 1.0);
    m_driverRotationConditioning.SetExponent(kDefaultExponent);

    frc::SmartDashboard::PutNumber("Speed Deadband", kDefaultDeadband);
    frc::SmartDashboard::PutNumber("Speed Output Offset", kDefaultOutputOffset);
    frc::SmartDashboard::PutNumber("Speed Exponent", kDefaultExponent);

    frc::SmartDashboard::PutNumber("Rotation Deadband", kDefaultDeadband);
    frc::SmartDashboard::PutNumber("Rotation Output Offset", kDefaultOutputOffset);
    frc::SmartDashboard::PutNumber("Rotation Exponent", kDefaultExponent);

    frc::SmartDashboard::PutBoolean("Update Conditioning", false);

    frc::SmartDashboard::PutBoolean(DriverCameraMode, false);

    frc::SmartDashboard::PutString("Robot State", "Traveling");

    frc::SmartDashboard::PutNumber(kCameraAngle, 23.3);
}

void Robot::RobotPeriodic() 
{
    if (m_skips % 47)
    {
        m_vision.SetCameraAngle(frc::SmartDashboard::GetNumber(kCameraAngle, 23.3));
        m_drivetrain.UpdateFromSmartDashboard();
        if (frc::SmartDashboard::GetBoolean("Update Conditioning", false))
        {
            double ldb = frc::SmartDashboard::GetNumber("Speed Deadband", kDefaultDeadband);
            double loo = frc::SmartDashboard::GetNumber("Speed Output Offset", kDefaultOutputOffset);
            double lex = frc::SmartDashboard::GetNumber("Speed Exponent", kDefaultExponent);

            double rdb = frc::SmartDashboard::GetNumber("Rotation Deadband", kDefaultDeadband);
            double roo = frc::SmartDashboard::GetNumber("Rotation Output Offset", kDefaultOutputOffset);
            double rex = frc::SmartDashboard::GetNumber("Rotation Exponent", kDefaultExponent);

            m_driverSpeedConditioning.SetDeadband(ldb);
            m_driverSpeedConditioning.SetRange(loo, 1.0);
            m_driverSpeedConditioning.SetExponent(lex);

            m_driverRotationConditioning.SetDeadband(rdb);
            m_driverRotationConditioning.SetRange(roo, 1.0);
            m_driverRotationConditioning.SetExponent(rex);
        }
    }
    if (true /*m_skips % 51*/)
    {
        m_shooter.OnRobotPeriodic();
    }
    if (m_skips % 53)
    {
        m_intake.OnRobotPeriodic();
    }
    if (m_skips % 57)
    {
        m_climb.OnRobotPeriodic();
    }

    // m_colorWheel.UpdateColorSensorValues(m_skips);

    if (m_skips % 33)
    {
        m_vision.OnRobotPeriodic();
    }

    // Increment the m_skips variable for counting
    m_skips++;
}


void Robot::DisabledInit() 
{
    m_bling.SendMessage(Bling::BlingMessage::on);
}

void Robot::AutonomousInit()
{
    m_bling.SendMessage(Bling::BlingMessage::autonomous);
    m_shooter.SetAutoKp();
    m_stateManager.OnAutoInit();
    m_autonomous.OnAutoInit();
    m_vision.SetCameraMode(CameraMode::VisionProcessing);
}

void Robot::AutonomousPeriodic()
{
    m_autonomous.Periodic();
    m_stateManager.OnAutoPeriodic();
}

void Robot::TeleopInit()
{
    m_bling.SendMessage(Bling::BlingMessage::spin);
    m_stateManager.Reset();
    m_shooter.SetTeleopKp();
    m_vision.SetCameraMode(CameraMode::VisionProcessing);
    // remove from competition code
    m_intake.SetIntakeSpeed(0);
    m_intake.SetConveyorSpeed(0);
    m_intake.SetKickerSpeed(0);
}

void Robot::TeleopPeriodic() 
{
    auto    cameraMode{ frc::SmartDashboard::GetBoolean(DriverCameraMode, false) };

    m_vision.SetCameraMode(cameraMode ? CameraMode::Driver : CameraMode::VisionProcessing);

    if (m_coPilot.GetRawButtonPressed(kTurnToTarget))
    {
        // read from camera
        m_origTx = m_vision.GetTargetXAngleDegrees();
    }
    else if (m_coPilot.GetRawButtonReleased(kTurnToTarget))
    {
        m_origTx = 0;
    }
    else if (m_origTx != 0)
    {
        if (m_drivetrain.TryTurnToTargetAngle(m_vision.GetTargetXAngleDegrees()) == false)
        {
            m_origTx = 0;
        }
    }
    else
    {
        double y = -m_driverJoystick.GetY();
        double x = m_driverJoystick.GetX();
        m_drivetrain.CurvatureDrive(
            m_driverSpeedConditioning.Condition(-m_driverJoystick.GetY()),
            m_driverRotationConditioning.Condition(m_driverJoystick.GetTwist()),
            m_quickTurn.Get());
    }
    
    m_shooter.SetSpeedFromThrottle(m_coPilot.GetThrottle());

    if (m_coPilot.GetRawButtonPressed(kSetAngle))
    {
        m_shooter.SetAngle(!m_shooter.GetAngle());
    }

    // When Firing Done
    if (m_coPilot.GetTriggerReleased())
    {
        m_stateManager.StartState(States::Traveling);
    }
    else if (m_shooter.CloseToSpeed() && m_coPilot.GetTriggerPressed())
    {
        m_stateManager.StartState(States::Firing);
    }

    if (m_coPilot.GetRawButtonPressed(kIntakeToggle))
    {
        if (m_intake.IsIntakeRunning())
        {
            m_stateManager.StartState(States::Traveling);
        }
        else
        {
            m_stateManager.StartState(States::Loading);
        }
    }

    // if (m_coPilot.GetRawButtonPressed(kEngageColorWheel))
    // {
    //     if (m_colorWheel.IsColorWheelEngaged())
    //     {
    //         m_stateManager.StartState(Robot::States::Traveling);
    //     }
    //     else
    //     {
    //         m_stateManager.StartState(Robot::States::ColorWheel);
    //     }
    // }

    // if (m_coPilot.GetRawButtonPressed(kClimbExtend))
    // {
    //     m_stateManager.StartState(States::Climbing);
    // }

    m_stateManager.OnTeleopPeriodic();
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
