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
import frc.robot.LimeLight;
import frc.robot.Robot;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.subsystems.ShooterTiltSubsystem;

public class PositionTiltandLock extends CommandBase {
  /**
   * Creates a new PositionTilt.
   * 
   * Limelight Field-of-View: 59.6 x 49.7 degrees
   */

  private final ShooterTiltSubsystem tilt;
  private final LimeLight limelight;

  private double turns;
  private double simStartTime;
  private int onTarget;
  private int targetSeen;

  public PositionTiltandLock(ShooterTiltSubsystem tilt, LimeLight limelight, double turns) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tilt = tilt;
    this.limelight = limelight;
    this.turns = turns;
    addRequirements(tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("CMDTIALRng", true);
    simStartTime = Timer.getFPGATimestamp();
    onTarget = 0;
    targetSeen = 0;
    if (turns > HoodedShooterConstants.TILT_MAX_TURNS)
      turns = HoodedShooterConstants.TILT_MAX_TURNS;
    if (turns < HoodedShooterConstants.TILT_MIN_TURNS)
      turns = HoodedShooterConstants.TILT_MIN_TURNS;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * Used in autonomous to move the tilt axis and lock it into position with the
     * limelight. In order to cut down on invalid target detections it will only
     * look at vision when it is inside 2 turns of the commanded position and the
     * limelight vertical target is within a range.
     * 
     * Turns are set iin the SetShootData command
     * 
     * The command will exit when locked on vision so autonomous can continue.
     * 
     * When exiting the default tilt axix command takes over locking the axis in
     * position
     * 
     */

    if (limelight.getIsTargetFound())
      targetSeen++;
    else
      targetSeen = 0;
    if (tilt.getTiltPosition() < 1 || !limelight.getIsTargetFound()) {
      tilt.positionTilttoTurns(turns);
    } else {
      tilt.lockTiltToVision(limelight.getdegVerticalToTarget());
    }
    if (limelight.getVertOnTarget())
      onTarget++;
    else
      onTarget = 0;

    SmartDashboard.putNumber("OT", onTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("CMDTIALRng", false);
    double lasttPosition = tilt.getTiltPosition();
    tilt.positionCommandTurns = lasttPosition;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tilt.getTiltPosition() > 1 && onTarget >= 2
        || Robot.isSimulation() && Timer.getFPGATimestamp() > simStartTime + 2;
  }
}
