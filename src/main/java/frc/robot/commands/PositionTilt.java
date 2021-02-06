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
import frc.robot.subsystems.ShooterTiltSubsystem;

public class PositionTilt extends CommandBase {
  /**
   * Creates a new PositionTilt.
   */

  private final ShooterTiltSubsystem tilt;

  private final double turns;
  private double simStartTime;

  public PositionTilt(ShooterTiltSubsystem tilt, double turns) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tilt = tilt;
    this.turns = turns;
    addRequirements(tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    simStartTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tilt.positionTilttoTurns(turns);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // tilt.positionCommandTurns = tilt.getTiltPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(turns - tilt.getTiltPosition()) < 1.5 && tilt.getTiltSpeed() < .25
        || tilt.m_reverseLimit.get() && tilt.getTiltOut() <= 0
        || Robot.isSimulation() && Timer.getFPGATimestamp() > simStartTime + 2;

  }
}
