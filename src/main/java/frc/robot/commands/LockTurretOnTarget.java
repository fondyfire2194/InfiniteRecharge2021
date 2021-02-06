/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class LockTurretOnTarget extends CommandBase {
  /**
   * Creates a new PIDLockTurret.
   */
  ShooterTurretSubsystem turret;
  LimeLight limelight;

  private boolean targetSeen;
  private boolean autoLookForTarget;
  private boolean teleopLookForTarget;
  private boolean targetAvailable;

  public LockTurretOnTarget(ShooterTurretSubsystem turret, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.limelight = limelight;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    autoLookForTarget = DriverStation.getInstance().isAutonomous()
        && Math.abs(turret.commandTurns - turret.getTurretPosition()) < 50
        && Math.abs(limelight.getdegRotationToTarget()) < 10.;

    teleopLookForTarget = DriverStation.getInstance().isOperatorControl()
        && Math.abs(limelight.getdegRotationToTarget()) < 10.;

    targetAvailable = limelight.getIsTargetFound();// && (autoLookForTarget || teleopLookForTarget)
    // && !turret.changeLocked;

    if (targetAvailable) {

      targetSeen = true;

      turret.lockTurretToVision(limelight.getdegRotationToTarget());

    } else {

      if (targetSeen) {

        turret.commandTurns = turret.getTurretPosition();

        targetSeen = false;
      }
      turret.positionTurretToTurns();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.commandTurns = turret.getTurretPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return limelight.getHorOnTarget() && DriverStation.getInstance().isAutonomous();
  }
}
