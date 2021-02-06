/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraightJoystick extends CommandBase {
  /**
   * Creates a new DriveStraightJoystick.
   */
  private final DriveSubsystem drive;
  private double startAngle;
  private DoubleSupplier forward;

  public DriveStraightJoystick(DriveSubsystem drive, DoubleSupplier forward) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.forward = forward;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startAngle = drive.getGyroYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    drive.arcadeDrive(forward.getAsDouble(), (startAngle - drive.getGyroYaw()) * Pref.getPref("DrStKp"));
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
