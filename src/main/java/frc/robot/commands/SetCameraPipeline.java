
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.LimeLight;
import frc.robot.LimelightControlMode.LedMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

public class SetCameraPipeline extends InstantCommand {
  private final LimeLight limelight;
  private final int number;

  public SetCameraPipeline(LimeLight limelight, int number) {
    this.limelight = limelight;

    this.number = number;

    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    limelight.setPipeline(number);
    limelight.setLEDMode(LedMode.kpipeLine);


  }
}
