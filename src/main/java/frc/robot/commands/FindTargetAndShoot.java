/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimeLight;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodedShooterSubsystem;
import frc.robot.subsystems.ShooterTiltSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FindTargetAndShoot extends SequentialCommandGroup {
  /**
   * Creates a new FindTargetAndShoot.
   */
  public FindTargetAndShoot(HoodedShooterSubsystem shooter, ShooterTurretSubsystem turret,
      CellTransportSubsystem transport, ShooterTiltSubsystem tilt, DriveSubsystem drive, LimeLight limelight,
      Compressor compressor, int number) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super( new TurretFindTarget(turret, limelight, true),
        new TiltFindTarget(tilt, limelight, true),
        new ParallelCommandGroup(new LockTurretOnTarget(turret, limelight), new LockTiltOnTarget(tilt, limelight))
            .deadlineWith(new StartShooter(shooter)),
        new ShootCells(shooter, transport, compressor,1,1));
  }
}
