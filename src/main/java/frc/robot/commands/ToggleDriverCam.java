/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.LimeLight;
import frc.robot.LimelightControlMode.CamMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ToggleDriverCam extends InstantCommand {

  private final LimeLight limelight;

  public ToggleDriverCam(LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (limelight.getCamMode() == CamMode.kdriver)
      limelight.setCamMode(CamMode.kvision);
    else
      limelight.setCamMode(CamMode.kdriver);
  }
}
