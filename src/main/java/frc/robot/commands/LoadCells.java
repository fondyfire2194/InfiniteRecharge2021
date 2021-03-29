/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CellTransportSubsystem;

public class LoadCells extends CommandBase {
  /**
   * Creates a new LoadCells.
   */
  CellTransportSubsystem transport;

  public LoadCells(CellTransportSubsystem transport) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.transport = transport;
    addRequirements(transport);
  };

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!transport.powerCellAtIntake.get())
      transport.runLeftBeltMotor(.25);
    else
      transport.runLeftBeltMotor(0);
    ;
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
