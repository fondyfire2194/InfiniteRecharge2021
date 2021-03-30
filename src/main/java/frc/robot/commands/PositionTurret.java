// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterTiltSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class PositionTurret extends CommandBase {
  /** Creates a new PositionTilt. */

  private final ShooterTurretSubsystem m_turret;

  private double m_position;

  private double m_endpoint;

  public PositionTurret(ShooterTurretSubsystem turret, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endpoint = m_position *8888;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.motionMagic(m_endpoint);
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
