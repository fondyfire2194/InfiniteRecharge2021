/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class PositionTurret extends CommandBase {
  /**
   * Creates a new PositionTilt.
   */

  private final ShooterTurretSubsystem turret;

  private final double angle;
  private double turns;
  private double simStartTime;

  public PositionTurret(ShooterTurretSubsystem turret, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;

    this.angle = angle;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turns = angle / HoodedShooterConstants.TURRET_ENCODER_DEG_PER_REV;
    if (turns > HoodedShooterConstants.TURRET_MAX_TURNS)
      turns = HoodedShooterConstants.TURRET_MAX_TURNS;
    if (turns < HoodedShooterConstants.TURRET_MIN_TURNS)
      turns = HoodedShooterConstants.TURRET_MIN_TURNS;
    simStartTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.positionTurret(turns);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(turns - turret.getTurretPosition()) < 2 && turret.getTurretSpeed() < .1
        || Robot.isSimulation() && Timer.getFPGATimestamp() > simStartTime + 2;

  }
}
