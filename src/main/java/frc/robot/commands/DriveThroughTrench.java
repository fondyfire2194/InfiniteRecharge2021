/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveThroughTrench extends CommandBase {
  /**
   * Creates a new DriveThroughTrench.
   */
  private final DriveSubsystem drive;
  private final Joystick controller;

  private final double requiredSensorDistance = .75;
  private final double sensorDistanceFromRobotCenter = .25;

  private final double secondSensorDistanceFromFirst = .4;

  double sensor1Distance = 0;
  double sensor2Distance = 0;
  private double robotAngle;

  public DriveThroughTrench(DriveSubsystem drive, Joystick controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.controller = controller;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double sensorDifference = sensor1Distance - sensor2Distance;

    double x = sensorDifference / secondSensorDistanceFromFirst;

    double distanceAtAngle = requiredSensorDistance + sensorDistanceFromRobotCenter * Math.sin(x);
    double angleRads = Math.asin(x);

    // if (drive.m_fRangeSensor.getRange() < requiredDistance)
    // drive.arcadeDrive(-controller.getY(), controller.getTwist() / 3);

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
