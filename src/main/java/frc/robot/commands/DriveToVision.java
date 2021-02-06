/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.Pref;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToVision extends CommandBase {
  /**
   * Creates a new DriveStraightJoystick.
   */
  private final DriveSubsystem drive;
  private DoubleSupplier forward;
  private final LimeLight limelight;
  private final int pipeline;

  public <Limelight> DriveToVision(DriveSubsystem drive, LimeLight limelight, DoubleSupplier forward, int pipeline) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.limelight = limelight;
    this.forward = forward;
    this.pipeline = pipeline;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setPipeline(pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    drive.arcadeDrive(forward.getAsDouble(), limelight.getdegRotationToTarget() * Pref.getPref("DrVKp"));
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
