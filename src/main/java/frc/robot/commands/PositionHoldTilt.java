/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterTiltSubsystem;

public class PositionHoldTilt extends CommandBase {
  /**
   * Creates a new PositionTilt.
   */

  private final ShooterTiltSubsystem tilt;

  public PositionHoldTilt(ShooterTiltSubsystem tilt) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tilt = tilt;
    addRequirements(tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tilt.positionTilttoTurns();
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
