/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimeLight;
import frc.robot.commands.PositionHoldTilt;
import frc.robot.commands.PositionHoldTurret;
import frc.robot.commands.PositionTilt;
import frc.robot.commands.PositionTiltandLock;
import frc.robot.commands.PositionTurret;
import frc.robot.commands.PositionTurretandLock;
import frc.robot.commands.SetCameraPipeline;
import frc.robot.commands.ShootCells;
import frc.robot.commands.StartShooter;
import frc.robot.commands.TiltMoveToReverseLimit;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodedShooterSubsystem;
import frc.robot.subsystems.ShooterTiltSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;
import frc.robot.trajectories.FondyFireTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoNH1 extends SequentialCommandGroup {
  /**
   * Creates a new Auto0.
   */
  private final static int pipeline = 1;
  private final static double tiltTurns = 5;
  private final static double turretAngle = 0;
  private final static double shootSpeed = 3700;
  private final static double shootTime = 5;

  public AutoNH1(HoodedShooterSubsystem shooter, ShooterTurretSubsystem turret, ShooterTiltSubsystem tilt,
      CellTransportSubsystem transport, DriveSubsystem drive, LimeLight limelight, FondyFireTrajectory s_trajectory,
      Compressor compressor) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());

    super(new TiltMoveToReverseLimit(tilt), new SetCameraPipeline(limelight, pipeline), new StartShooter(shooter),
        new PositionTiltandLock(tilt, limelight, tiltTurns), new PositionTurretandLock(turret, limelight, turretAngle),
        s_trajectory.getRamsete(s_trajectory.centerStart).andThen(() -> drive.tankDriveVolts(0, 0)),
        new ParallelCommandGroup(new ShootCells(shooter, transport, compressor, shootSpeed, shootTime)
            .deadlineWith(new ParallelCommandGroup(new PositionHoldTilt(tilt)), new PositionHoldTurret(turret))),
        new ParallelCommandGroup(new PositionTilt(tilt, -1), new PositionTurret(turret, 0)));
  }
}