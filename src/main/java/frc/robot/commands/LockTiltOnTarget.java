/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.LimeLight;
import frc.robot.subsystems.ShooterTiltSubsystem;

public class LockTiltOnTarget extends CommandBase {
  /**
   * Creates a new PIDLockTurret.
   */
  ShooterTiltSubsystem tilt;
  LimeLight limelight;
  private boolean targetWasSeen;
  private boolean autoLookForTarget;
  private boolean teleopLookForTarget;
  private boolean targetAvailable;
  private boolean goPlus;
  private boolean goMinus;

  private double jogSpeed;

  public LockTiltOnTarget(ShooterTiltSubsystem tilt, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tilt = tilt;
    this.limelight = limelight;
    addRequirements(tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.

  /**
   * Lock onto target. In auto tilt is commanded to center of target. Once it is
   * within 2 turns and the target is within a band (to help avoid bad targets)
   * lock on using a PID Controller
   * 
   * 
   * 
   */

  @Override
  public void execute() {

    autoLookForTarget = DriverStation.getInstance().isAutonomous()
        && Math.abs(tilt.positionCommandTurns - tilt.getTiltPosition()) < 2
        && Math.abs(limelight.getdegVerticalToTarget()) < 12.;

    teleopLookForTarget = DriverStation.getInstance().isOperatorControl()
        && Math.abs(limelight.getdegVerticalToTarget()) < 12.;

    targetAvailable = limelight.getIsTargetFound();// && (autoLookForTarget || teleopLookForTarget &&
                                                   // !tilt.changeLocked);

    if (targetAvailable) {

      targetWasSeen = true;
      // limelight.setSnapshot(Snapshot.kon);

      tilt.lockTiltToVision(limelight.getdegVerticalToTarget());

    }
    /**
     * Lost target need to lock on position loop at present value
     * 
     */
    if (!targetAvailable && targetWasSeen) {
      tilt.positionCommandTurns = tilt.getTiltPosition();
      targetWasSeen = false;
    }

    if (!targetAvailable && !targetWasSeen) {
      tilt.positionTilttoTurns();
    }
    /**
     * Looking for target triggered by driver in teleop only Driver needs to point
     * camera at target before starting
     * 
     */
    if (tilt.loookForTarget && !limelight.getIsTargetFound()) {
      if (goPlus ^ goMinus)
        goPlus = true;

      if (goMinus && tilt.getTiltPosition() < HoodedShooterConstants.TILT_MIN_TURNS + 1) {
        goPlus = true;
        goMinus = false;
        tilt.positionCommandTurns = HoodedShooterConstants.TILT_MAX_TURNS - 1;
      }
      if (goPlus && tilt.getTiltPosition() > HoodedShooterConstants.TILT_MAX_TURNS - 1) {
        goMinus = true;
        goPlus = false;
        tilt.positionCommandTurns = HoodedShooterConstants.TILT_MIN_TURNS + 1;
      }

      tilt.positionTilttoTurns();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tilt.positionCommandTurns = tilt.getTiltPosition();
    // limelight.setSnapshot(Snapshot.koff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return limelight.getVertOnTarget() && DriverStation.getInstance().isAutonomous();
  }
}
