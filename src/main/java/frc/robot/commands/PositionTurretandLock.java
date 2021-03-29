/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.LimeLight;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class PositionTurretandLock extends CommandBase {
  /**
   * Creates a new PositionTilt.
   */

  private final ShooterTurretSubsystem turret;
  private final LimeLight limelight;

  private double angle;
  private double turns;
  private double simStartTime;
  private int onTarget;

  public PositionTurretandLock(ShooterTurretSubsystem turret, LimeLight limelight, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.limelight = limelight;
    this.angle = angle;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (angle > HoodedShooterConstants.TURRET_MAX_ANGLE)
      angle = HoodedShooterConstants.TURRET_MAX_ANGLE;
    if (angle < HoodedShooterConstants.TURRET_MIN_ANGLE)
      angle = HoodedShooterConstants.TURRET_MIN_ANGLE;
    SmartDashboard.putBoolean("CMDTUALRng", true);
    simStartTime = Timer.getFPGATimestamp();
    onTarget = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!limelight.getIsTargetFound()) {
      turret.positionTurretToAngle(angle);

    } else {
      turret.lockTurretToVision(limelight.getdegRotationToTarget());
    }
    if (limelight.getVertOnTarget())
      onTarget++;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("CMDTUALRng", false);
    turret.commandAngle = turret.getTurretAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return onTarget > 2;
  }
}
