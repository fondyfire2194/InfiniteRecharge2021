// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterTiltSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class TiltJog extends CommandBase {
  /** Creates a new TurretJog. */
  private final ShooterTiltSubsystem m_tilt;
  private double m_speed;

  public TiltJog(ShooterTiltSubsystem tilt, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tilt = tilt;
    m_speed = speed;
    addRequirements(m_tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_tilt.jogTilt(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tilt.holdPositionDegrees = m_tilt.getTiltAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
