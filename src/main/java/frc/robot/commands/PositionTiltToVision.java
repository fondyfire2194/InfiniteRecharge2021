// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.subsystems.ShooterTiltSubsystem;

public class PositionTiltToVision extends CommandBase {
  /** Creates a new PositionTilt. */

  private final ShooterTiltSubsystem m_tilt;

  private final LimeLight m_limelight;

  private double m_position;

  private double m_endpoint;

  public PositionTiltToVision(ShooterTiltSubsystem tilt, double position, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tilt = tilt;
    m_position = position;
    m_limelight = limelight;
    addRequirements(m_tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endpoint = m_position * m_tilt.encoderCountsPerDegree;
    m_tilt.visionCorrection = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_limelight.getIsTargetFound()) {
      m_tilt.visionCorrection = 0;

    } else {
      m_tilt.visionCorrection = m_limelight.getdegVerticalToTarget();
    }
    m_tilt.motionMagic(m_endpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
