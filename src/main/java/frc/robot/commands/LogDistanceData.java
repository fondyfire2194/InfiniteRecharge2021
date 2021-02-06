/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterTiltSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;

public class LogDistanceData extends CommandBase {
  /**
   * Creates a new LogDistanceData.
   */
  public final String[] names = { "Step", "LeftMeters", "RightMeters", "AvgeMeters", "LeftMPersec", "GyroAngle", "Tilt",
      "Turret", "TargetSeen", "BoxHeight", "BoxWidth", "BoxArea", "HorToTarget", "VertToTarget" };
  public static String[] units = { "Number", "Meters", "Meters", "Meters", "MPS", "Degrees", "Turns", "Turns", "OnOff",
      "Pixels", "Pixels", "SqOPixels", "Degrees", "Degrees" };
  private int loopCtr;
  private boolean fileOpenNow;
  private int step;
  private final DriveSubsystem drive;
  private final LimeLight limelight;
  private final ShooterTurretSubsystem turret;
  private final ShooterTiltSubsystem tilt;
  private int lastPixel;

  public LogDistanceData(DriveSubsystem drive, ShooterTurretSubsystem turret, ShooterTiltSubsystem tilt,
      LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.limelight = limelight;
    this.turret = turret;
    this.tilt = tilt;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.simpleCSVLogger.init("Distance", "Data", names, units);
    loopCtr = 0;
    fileOpenNow = false;
    step = 0;
    lastPixel = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // allow i second for file to be opened
    if (!fileOpenNow)
      loopCtr++;
    if (loopCtr > 500) {
      fileOpenNow = true;
      loopCtr = 0;
    }
    // log data every 2 pixels
    if (fileOpenNow)
      loopCtr++;
    if (loopCtr >= 25) {
      loopCtr = 0;
      step++;
      double targetSeen = 0;
      if (limelight.getIsTargetFound())
        targetSeen = 1;

      drive.simpleCSVLogger.writeData((double) step, drive.getLeftDistanceMeters(), drive.getRightDistanceMeters(),
          drive.getAverageDistanceMeters(), drive.getLeftEncoderMps(), drive.getGyroAngle(), tilt.getTiltPosition(),
          turret.getTurretPosition(), targetSeen, limelight.getBoundingBoxHeight(), limelight.getBoundingBoxWidth(),
          limelight.getTargetArea(), limelight.getdegRotationToTarget(), limelight.getdegVerticalToTarget());
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.simpleCSVLogger.close();
    drive.endFile = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drive.endFile;
  }
}
