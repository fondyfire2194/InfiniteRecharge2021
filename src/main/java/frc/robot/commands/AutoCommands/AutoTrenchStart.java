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
import frc.robot.commands.StartRearIntake;
import frc.robot.commands.StartShooter;
import frc.robot.commands.StopRearIntake;
import frc.robot.commands.TiltMoveToReverseLimit;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodedShooterSubsystem;
import frc.robot.subsystems.RearIntakeSubsystem;
import frc.robot.subsystems.ShooterTiltSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;
import frc.robot.trajectories.FondyFireTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoTrenchStart extends SequentialCommandGroup {
  /**
   * Creates a new Auto0.
   */
  private final static int pipeline = 1;
  private final static double tiltTurns = 5;
  private final static double tiltTurns1 = 5;
  private final static double turretAngle = 0;
  private final static double turretAngle1 = 0;
  private final static double shootSpeed = 3700;
  private final static double shootTime = 5;
  private final static double shootTime1 = 5;

  public AutoTrenchStart(HoodedShooterSubsystem shooter, ShooterTurretSubsystem turret, ShooterTiltSubsystem tilt,
      CellTransportSubsystem transport, DriveSubsystem drive, RearIntakeSubsystem rearIntake, LimeLight limelight,
      FondyFireTrajectory s_trajectory, Compressor compressor) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    /**
     * Start position is in line with trench center line preloaded with 3 cells.
     * 
     * 1) Reverse to pick up 2 more cells, run intake, plus shooter at lower speed,
     * preposition tilt and turret during move
     * 
     * 2) Lock on target with vision, keeping intake down and running
     * 
     * 3) Lock on target with position and shoot 5 cells
     * 
     * 4) Lock on target with vision and reverse to pick up remaining 2 cells
     * 
     * 5) Lock on target with position and shoot 5 cells
     * 
     */

    super(new TiltMoveToReverseLimit(tilt), new SetCameraPipeline(limelight, pipeline), new StartShooter(shooter),
        new StartRearIntake(rearIntake),
        new ParallelCommandGroup(new PositionTurret(turret, turretAngle), new PositionTilt(tilt, tiltTurns),
            s_trajectory.getRamsete(s_trajectory.trenchStartOne).andThen(() -> drive.tankDriveVolts(0, 0))),

        new ParallelCommandGroup(new PositionTiltandLock(tilt, limelight, tiltTurns),
            new PositionTurretandLock(turret, limelight, turretAngle)),

        new ParallelCommandGroup(new ShootCells(shooter, transport, compressor, shootSpeed, shootTime)
            .deadlineWith(new ParallelCommandGroup(new PositionHoldTilt(tilt)), new PositionHoldTurret(turret))),

        new ParallelCommandGroup(new PositionTurret(turret, turretAngle), new PositionTilt(tilt, tiltTurns),
            s_trajectory.getRamsete(s_trajectory.trenchStartTwo).andThen(() -> drive.tankDriveVolts(0, 0))),
        new StopRearIntake(rearIntake),

        new ParallelCommandGroup(new PositionTiltandLock(tilt, limelight, tiltTurns1),
            new PositionTurretandLock(turret, limelight, turretAngle1)),

        new ParallelCommandGroup(new ShootCells(shooter, transport, compressor, shootSpeed, shootTime1)
            .deadlineWith(new ParallelCommandGroup(new PositionHoldTilt(tilt)), new PositionHoldTurret(turret))),

        new ParallelCommandGroup(new PositionTilt(tilt, -1), new PositionTurret(turret, 0)));

  }
}