/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

public class PositionControlPanel extends CommandBase {
  /**
   * Creates a new PositionControlPanel.
   */
  private final ControlPanelSubsystem cp;
  private int currentColor;
  private int targetColor;
  private int wheelDifference;

  public PositionControlPanel(ControlPanelSubsystem cp) {

    this.cp = cp;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(cp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentColor = cp.colorNumberFiltered;
    targetColor = cp.gameColorNumber + 2;// field sensor is 2 colors off robot sensor
    if (targetColor > 4)
      targetColor -= 4;
    wheelDifference = Math.abs(targetColor - currentColor);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (wheelDifference == 3) {
      cp.turnWheelMotor(.1);
    } else {
      cp.turnWheelMotor(-.1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cp.turnWheelMotor(0.);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cp.colorNumberFiltered == targetColor;
  }
}
