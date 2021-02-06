/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.Robot;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.subsystems.ShooterTiltSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class TiltFindTarget extends CommandBase {
  /**
   * Creates a new LockOnTarget.
   */

  private final ShooterTiltSubsystem tilt;
  private final LimeLight limelight;
  private boolean direction;
  private boolean goPlus;
  private boolean goMinus;
  private boolean inPosition;

  private double jogSpeed;
  private double startTime;

  public TiltFindTarget(ShooterTiltSubsystem tilt, LimeLight limelight, boolean direction) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.tilt = tilt;
    this.limelight = limelight;
    this.direction = direction;

    addRequirements(tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    startTime = Timer.getFPGATimestamp();
    goPlus = direction;
    goMinus = direction;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (goPlus ^ goMinus)
      goPlus = true;
    jogSpeed = -.05;
    if (goPlus)
      jogSpeed = .05;

    if (goMinus && tilt.getTiltPosition() < HoodedShooterConstants.TILT_MIN_TURNS) {
      goPlus = true;
      goMinus = false;
    }
    if (goPlus && tilt.getTiltPosition() > HoodedShooterConstants.TILT_MAX_TURNS) {
      goMinus = true;
      goPlus = false;
    }
    tilt.jogTilt(jogSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() > startTime + 10
        || limelight.getIsTargetFound() && Math.abs(limelight.getdegRotationToTarget()) < 10;
  }
}
