/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <subsystems/Drivetrain.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <AngleConversion.h>

Drivetrain::Drivetrain() 
{
    m_leftFollower1.Follow(m_leftPrimary);
    m_leftFollower2.Follow(m_leftPrimary);
    m_rightFollower1.Follow(m_rightPrimary);
    m_rightFollower2.Follow(m_rightPrimary);

    m_leftPrimary.SetInverted(false);
    m_rightPrimary.SetInverted(false);
    m_differentialDrive.SetRightSideInverted(true);

    SetupSparkMax(&m_leftPrimary);
    SetupSparkMax(&m_rightPrimary);
    SetupSparkMax(&m_leftFollower1);
    SetupSparkMax(&m_leftFollower2);
    SetupSparkMax(&m_rightFollower1);
    SetupSparkMax(&m_rightFollower2);
}

void Drivetrain::SetupSparkMax(rev::CANSparkMax* controller)
{
    controller->ClearFaults();
    controller->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    controller->SetSmartCurrentLimit(kCurrentLimit);
    controller->SetOpenLoopRampRate(kOpenLoopRampRate);
}

// void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) 
// {
//     // Don't know what to do here

//     // m_leftPID.SetReference(static_cast<double>(speeds.left), rev::ControlType::kVelocity);
//     // m_rightPID.SetReference(static_cast<double>(speeds.right), rev::ControlType::kVelocity);
// }

// void Drivetrain::SetSpeeds(double left, double right)
// {
//     // m_leftPID.SetReference(left, rev::ControlType::kVelocity);
//     // m_rightPID.SetReference(right, rev::ControlType::kVelocity);
// }

void Drivetrain::CurvatureDrive(double speed, double rotation, bool quickTurn)
{
    m_differentialDrive.CurvatureDrive(speed, rotation, quickTurn);
}

void Drivetrain::TankDrive(double left, double right)
{
    m_differentialDrive.TankDrive(left, right);
}

void Drivetrain::InitalShowToSmartDashboard()
{
    m_leftPID.SetFF(0.0005);
    m_rightPID.SetFF(0.0005);
    // Debug Enable
    frc::SmartDashboard::PutBoolean(kDebug, m_debugEnable);
    // PID
    frc::SmartDashboard::PutNumber(kPGain, m_leftPID.GetP());
    frc::SmartDashboard::PutNumber(kIGain, m_leftPID.GetI());
    frc::SmartDashboard::PutNumber(kFF, m_leftPID.GetFF());
    frc::SmartDashboard::PutNumber(kIZone, m_leftPID.GetIZone());
    // Turn to Angle parameters
    frc::SmartDashboard::PutNumber(kAutoKp, kDefaultAutoKp);
    frc::SmartDashboard::PutNumber(kAutoLimitAngle, kDefaultLimitAngle);
    frc::SmartDashboard::PutNumber(kAutoMinSpeed, kDefaultMinSpeed);
    frc::SmartDashboard::PutNumber(kNavxAngle, 0);
}

void Drivetrain::UpdateFromSmartDashboard()
{    
    m_debugEnable = frc::SmartDashboard::GetBoolean(kDebug, false);

    if (m_debugEnable == false) return;

    frc::SmartDashboard::PutNumber(kNavxAngle, GetAngle());

    // Get the values only once to optimize for speed
    auto currentP = m_leftPID.GetP();
    auto currentI = m_leftPID.GetI();
    auto currentFF = m_leftPID.GetFF();
    auto currentIZone = m_leftPID.GetIZone();

    auto myP = frc::SmartDashboard::GetNumber(kPGain, currentP);
    auto myI = frc::SmartDashboard::GetNumber(kIGain, currentI);
    auto myFF = frc::SmartDashboard::GetNumber(kFF, currentFF);
    auto myIZone = frc::SmartDashboard::GetNumber(kIZone, currentIZone);
    if(fabs(myP - currentP) > kCloseToSameValue)
    {
        m_rightPID.SetP(myP);
        m_leftPID.SetP(myP);
    }
    if(fabs(myI - currentI) > kCloseToSameValue)
    {
        m_rightPID.SetI(myI);
        m_leftPID.SetI(myI);
    }
    if(fabs(myFF - currentFF) > kCloseToSameValue)
    {
        m_rightPID.SetFF(myFF);
        m_leftPID.SetFF(myFF);
    }
    if(fabs(myIZone - currentIZone) > kCloseToSameValue)
    {
        m_rightPID.SetIZone(myIZone);
        m_leftPID.SetIZone(myIZone);
    }

    m_autoKp = frc::SmartDashboard::GetNumber(kAutoKp, kDefaultAutoKp);
    m_autoLimitAngle = frc::SmartDashboard::GetNumber(kAutoLimitAngle, kDefaultLimitAngle);
    m_autoMinSpeed = frc::SmartDashboard::GetNumber(kAutoMinSpeed, kDefaultMinSpeed);
}

bool Drivetrain::TryTurnToTargetAngle(double tx)
{
    // auto currentAngle = m_navX.GetAngle();
    if (std::fabs(tx) < m_autoLimitAngle)
    {
        CurvatureDrive(0.0, 0.0, false);
        return false;
    }

    // auto rotationMagnitude = tx;//targetAngle - currentAngle;
    auto rotationSpeed = 0.0;
    // is tx negative, which means turn left
    if (tx < 0)
    {
        // turn left -> counter clockwise
        rotationSpeed = std::fmin(-m_autoMinSpeed, tx * m_autoKp);
    }
    else
    {
        // turn right -> clockwise
        rotationSpeed = std::fmax(m_autoMinSpeed, tx * m_autoKp);
    }
    
    CurvatureDrive(0.0, rotationSpeed, true);

    return true;
}

double Drivetrain::GetAngle()
{
    return m_navX.GetAngle();
}

double Drivetrain::GetPostion()
{
    return m_rightEncoder.GetPosition();
}
