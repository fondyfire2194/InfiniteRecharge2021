// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class TurretJog extends CommandBase {
  /** Creates a new TurretJog. */
  private final ShooterTurretSubsystem m_turret;
  private double m_speed;

  public TurretJog(ShooterTurretSubsystem turret, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_speed = speed;
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.jogTurret(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.holdPositionDegrees = m_turret.getTurretAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
