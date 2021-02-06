/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class LogTrajData extends CommandBase {
  /**
   * Creates a new LogDistanceData.
   */
  public final String[] names = { "Step", "LeftMeters", "RightMeters", "AvgeMeters", "LeftMPersec", "RightMPersec",
      "GyroAngle" };
  public static String[] units = { "Number", "Meters", "Meters", "Meters", "MPS", "MPS","Degrees"};
  private int loopCtr;
  private boolean fileOpenNow;
  private int step;
  private final DriveSubsystem drive;

  public LogTrajData(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.simpleCSVLogger.init("Traj", "2020", names, units);
    loopCtr = 0;
    fileOpenNow = false;
    step = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // allow 1 second for file to be opened
    if (!fileOpenNow)
      loopCtr++;
    if (loopCtr > 50) {
      fileOpenNow = true;
      loopCtr = 0;
    }
    // log data every .1 seconds = 5 x 20 ms
    if (fileOpenNow)
      loopCtr++;
    if (loopCtr >= 5) {
      loopCtr = 0;
      step++;

      drive.simpleCSVLogger.writeData((double) step, drive.getLeftDistanceMeters(), drive.getRightDistanceMeters(),
          drive.getAverageDistanceMeters(), drive.getLeftEncoderMps(), drive.getRightEncoderMps(),
          drive.getGyroAngle());
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.simpleCSVLogger.close();
    drive.endTrajFile = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drive.endTrajFile;
  }
}
