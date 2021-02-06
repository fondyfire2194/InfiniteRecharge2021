/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightControlMode.LedMode;
import frc.robot.commands.AutoSwitchZoom;
import frc.robot.commands.TiltMoveToReverseLimit;
import frc.robot.commands.AutoCommands.Auto0;
import frc.robot.commands.AutoCommands.Auto1;
import frc.robot.commands.AutoCommands.Auto2;
import frc.robot.commands.AutoCommands.AutoNH1;
import frc.robot.commands.AutoCommands.AutoTrenchStart;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodedShooterSubsystem;
import frc.robot.subsystems.RearIntakeSubsystem;
import frc.robot.subsystems.ShooterTiltSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;
import frc.robot.trajectories.FondyFireTrajectory;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static SendableChooser<Integer> autoChooser;
  public static SendableChooser<Integer> startDelayChooser = new SendableChooser<>();
  private RobotContainer m_robotContainer;
  private double startTime;
  private double m_startDelay;
  private int choice;
  private boolean autoHasRun;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings.

    m_robotContainer = new RobotContainer();
    autoChooser = new SendableChooser<>();

    // Put
    // autonomous chooser on the dashboard.
    // The first argument is the root container
    // The second argument is whether logging and config should be given separate
    // tabs
    Shuffleboard.getTab("Pre-Round").add("Auto Commands", autoChooser).withSize(2, 1) // make the widget
        // 2x1
        .withPosition(0, 0); // place it in the top-left corner

    int place = 0;
    autoChooser.setDefaultOption("Center Start Retract Shoot", place);
    place = 1;
    autoChooser.addOption("Left Start Retract Shoot", place);
    place = 2;
    autoChooser.addOption("Right Start Retract Shoot", place);
    place = 3;
    autoChooser.addOption("Right Shoot Trench Pickup", place);
    place = 4;
    autoChooser.addOption("Cross Line", place);
    place = 5;
    autoChooser.addOption(("Example S Shape"), place);
    place = 6;
    autoChooser.addOption(("AutoNH1"), place);
    place = 7;
    autoChooser.addOption(("Trench Start"), place);

    Shuffleboard.getTab("Pre-Round").add("Auto Delay", startDelayChooser).withSize(2, 1) // make the widget
        // 2x1
        .withPosition(2, 0); //

    startDelayChooser.setDefaultOption("No Delay", 0);
    startDelayChooser.addOption("One Second", 1);
    startDelayChooser.addOption("Two Seconds", 2);
    startDelayChooser.addOption("Three Seconds", 3);
    startDelayChooser.addOption("four Seconds", 4);
    startDelayChooser.addOption("Five Seconds", 5);

    choice = 0;

    Shuffleboard.getTab("Pre-Round").add("Tilt Down OK", m_robotContainer.m_tilt.m_reverseLimit.get())
        .withWidget(BuiltInWidgets.kBooleanBox).withSize(2, 1).withPosition(2, 1);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    m_robotContainer.m_limelight.periodic();
    SmartDashboard.putNumber("MatchTime", DriverStation.getInstance().getMatchTime());
    SmartDashboard.putBoolean("LLVonTgt", m_robotContainer.m_limelight.getVertOnTarget());
    SmartDashboard.putBoolean("LLHonTgt", m_robotContainer.m_limelight.getHorOnTarget());
    SmartDashboard.putBoolean("LLTgt", m_robotContainer.m_limelight.getIsTargetFound());
    if (DriverStation.getInstance().getGameSpecificMessage() != null)

      SmartDashboard.putString("Game Message", DriverStation.getInstance().getGameSpecificMessage());

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    m_robotContainer.m_limelight.setPipeline(1);
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledPeriodic() {

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */

  public void autonomousInit() {

    Shuffleboard.startRecording();
    DriveSubsystem drive = m_robotContainer.m_robotDrive;
    RearIntakeSubsystem rearIntake = m_robotContainer.m_rearIntake;
    CellTransportSubsystem transport = m_robotContainer.m_transport;
    FondyFireTrajectory s_trajectory = m_robotContainer.s_trajectory;
    HoodedShooterSubsystem shooter = m_robotContainer.m_shooter;
    ShooterTiltSubsystem tilt = m_robotContainer.m_tilt;
    ShooterTurretSubsystem turret = m_robotContainer.m_turret;
    LimeLight limelight = m_robotContainer.m_limelight;
    Compressor compressor = m_robotContainer.m_compressor;
    drive.resetEncoders();
    drive.resetGyro();
    limelight.setLEDMode(LedMode.kpipeLine);
    // get delay time

    m_startDelay = (double) startDelayChooser.getSelected();

    SmartDashboard.putNumber("Delay", m_startDelay);
    // schedule the autonomous command
    // new StartShooter(shooter),
    choice = autoChooser.getSelected();

    SmartDashboard.putNumber("AutoChoice", choice);
    autoHasRun = false;

    switch (choice) {

    case 0:// in front of power port 0 shooter data index use pipeline 0 - no zoom

      m_robotContainer.m_robotDrive.resetAll();
      m_robotContainer.m_robotDrive.resetOdometry(s_trajectory.centerStart.getInitialPose());
      m_autonomousCommand = new Auto0(shooter, turret, tilt, transport, drive, limelight, s_trajectory, compressor);

      break;

    case 1:// on left of center robot as close as possible
      m_robotContainer.m_robotDrive.resetAll();
      m_robotContainer.m_robotDrive.resetOdometry(s_trajectory.leftStart.getInitialPose());
      m_autonomousCommand = new Auto1(shooter, turret, tilt, transport, drive, limelight, s_trajectory, compressor);

      break;

    case 2:// on right of center robot as close as possible
      m_robotContainer.m_robotDrive.resetAll();
      m_robotContainer.m_robotDrive.resetOdometry(s_trajectory.rightStart.getInitialPose());

      m_autonomousCommand = new Auto2(shooter, turret, tilt, transport, drive, limelight, s_trajectory, compressor);

      break;
    case 3:// start with left bumper in line with right of power port
      m_robotContainer.m_robotDrive.resetAll();

      m_autonomousCommand = new Auto1(shooter, turret, tilt, transport, drive, limelight, s_trajectory, compressor);

      break;

    case 4:// cross line
      m_robotContainer.m_robotDrive.resetAll();
      m_robotContainer.m_robotDrive.resetOdometry(s_trajectory.crossLine.getInitialPose());
      s_trajectory.getRamsete(s_trajectory.crossLine).andThen(() -> drive.tankDriveVolts(0, 0)).schedule();
      break;

    case 5:
      m_robotContainer.m_robotDrive.resetAll();
      m_robotContainer.m_robotDrive.resetOdometry(s_trajectory.example.getInitialPose());
      s_trajectory.getRamsete(s_trajectory.example).andThen(() -> drive.tankDriveVolts(0, 0)).schedule();
      break;

    case 6:
      m_robotContainer.m_robotDrive.resetAll();
      m_robotContainer.m_robotDrive.resetOdometry(s_trajectory.centerStart.getInitialPose());
      m_autonomousCommand = new AutoNH1(shooter, turret, tilt, transport, drive, limelight, s_trajectory, compressor);

      break;
    case 7:
      m_robotContainer.m_robotDrive.resetAll();
      m_robotContainer.m_robotDrive.resetOdometry(s_trajectory.trenchStartOne.getInitialPose());
      m_autonomousCommand = new AutoTrenchStart(shooter, turret, tilt, transport, drive, rearIntake, limelight,
          s_trajectory, compressor);

      break;
    case 8:

      break;
    default:

      break;
    }

    startTime = Timer.getFPGATimestamp();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    if (!autoHasRun && Timer.getFPGATimestamp() > startTime + m_startDelay && m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      autoHasRun = true;
    }

    double timeToStart = Math.round(startTime + m_startDelay - Timer.getFPGATimestamp());
    if (timeToStart >= 0)
      SmartDashboard.putNumber("Starting In", timeToStart);

    // CommandScheduler.getInstance().run();

    if (DriverStation.getInstance().getMatchTime() < 10)
      Shuffleboard.stopRecording();

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    CommandScheduler.getInstance().cancelAll();
    Shuffleboard.stopRecording();
    new TiltMoveToReverseLimit(m_robotContainer.m_tilt).schedule();
    autoHasRun = false;
    Shuffleboard.startRecording();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

}
