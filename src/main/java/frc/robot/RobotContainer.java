/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.CellTransportConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightControlMode.StreamType;
import frc.robot.commands.AutoSwitchZoom;
import frc.robot.commands.EndLogData;
import frc.robot.commands.LogDistanceData;
import frc.robot.commands.LogTrajData;
import frc.robot.commands.PositionTilt;
import frc.robot.commands.PositionTurret;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ResetPose;
import frc.robot.commands.ResetShooterAngle;
import frc.robot.commands.ResetShooterTilt;
import frc.robot.commands.SetCameraPipeline;
import frc.robot.commands.ShootCells;
import frc.robot.commands.StartRearIntake;
import frc.robot.commands.StartShooter;
import frc.robot.commands.StopShoot;
import frc.robot.commands.ToggleDriverCam;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodedShooterSubsystem;
import frc.robot.subsystems.RearIntakeSubsystem;
import frc.robot.subsystems.ShooterTiltSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;
import frc.robot.trajectories.FondyFireTrajectory;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
      // The robot's subsystems

      final DriveSubsystem m_robotDrive;

      public final LimeLight m_limelight;

      public final ShooterTurretSubsystem m_turret;

      public final ShooterTiltSubsystem m_tilt;

      public final ControlPanelSubsystem m_controlPanel;

      public final CellTransportSubsystem m_transport;

      public final HoodedShooterSubsystem m_shooter;

      public final RearIntakeSubsystem m_rearIntake;

      public final PowerDistributionPanel m_powerDistributionPanel;

      public final Compressor m_compressor;

      // The driver's controller
      private final Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
      public final XboxController gamepad = new XboxController(OIConstants.kCoDriverControllerPort);
      public final XboxController setupGamepad = new XboxController(OIConstants.kSetupControllerPort);
      public final XboxController shootBox = new XboxController(OIConstants.kShootBoxControllerPort);
      public final FondyFireTrajectory s_trajectory;
      //
      public static Preferences prefs;

      public static boolean autoSelected;

      // AutoCommands ac;// = new AutoCommands(m_robotDrive);
      public int shootPosition;

      /**
       * The container for the robot. Contains subsysems, OI devices, and commands.
       */
      public RobotContainer() {
            prefs = Preferences.getInstance();
            // Pref.deleteAllPrefs();
            // Pref.deleteUnused();
            // Pref.addMissing();

            m_robotDrive = new DriveSubsystem();
            s_trajectory = new FondyFireTrajectory(m_robotDrive);
            SmartDashboard.putData("Drive", m_robotDrive);
            m_turret = new ShooterTurretSubsystem();
            SmartDashboard.putData("Turret", m_turret);
            m_tilt = new ShooterTiltSubsystem();
            SmartDashboard.putData("Tilt", m_tilt);
            m_transport = new CellTransportSubsystem();
            SmartDashboard.putData("Celltransport", m_transport);

            m_limelight = new LimeLight();
            m_limelight.setStream(StreamType.kPiPSecondary);
            m_limelight.setPipeline(0);

            m_controlPanel = new ControlPanelSubsystem();

            m_shooter = new HoodedShooterSubsystem();

            m_rearIntake = new RearIntakeSubsystem();
            m_compressor = new Compressor();
            m_powerDistributionPanel = new PowerDistributionPanel(Constants.PDP);

            new AutoSwitchZoom(m_limelight, m_tilt, m_turret, m_shooter).schedule();

            // // //Configure the button bindings

            configureButtonBindings();
            ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
            driveTab.add("Reset Encoders", new InstantCommand(() -> m_robotDrive.resetEncoders()));
            driveTab.add("Reset Gyro",new ResetGyro(m_robotDrive));
            driveTab.add("Reset Pose",new ResetPose(m_robotDrive));
            // SmartDashboard.putData(new ResetEncoders(m_robotDrive));
            // SmartDashboard.putData(new ResetGyro(m_robotDrive));
            // SmartDashboard.putData(new ResetPose(m_robotDrive));

            SmartDashboard.putData(new ResetShooterAngle(m_turret));
            SmartDashboard.putData(new ResetShooterTilt(m_tilt));
            SmartDashboard.putData(new ShootCells(m_shooter, m_transport, m_compressor, 3000, 0));

            SmartDashboard.putData("TiltTo5", new PositionTilt(m_tilt, 4));
            SmartDashboard.putData("TiltTo)", new PositionTilt(m_tilt, -1.5));

            SmartDashboard.putData("TurretTo +10", new PositionTurret(m_turret, 10));// degrees
            SmartDashboard.putData("TurretTo -10", new PositionTurret(m_turret, -10));// degrees
            SmartDashboard.putData("TurretTo +0", new PositionTurret(m_turret, 0));// degrees

            SmartDashboard.putData("LogData", new LogDistanceData(m_robotDrive, m_turret, m_tilt, m_limelight));
            SmartDashboard.putData("EndLogData", new EndLogData(m_robotDrive));

            SmartDashboard.putData("LogTrajData", new LogTrajData(m_robotDrive));
            SmartDashboard.putData("EndTrajData", new EndLogData(m_robotDrive));

            SmartDashboard.putBoolean("CMDTIALRng", false);
            SmartDashboard.putBoolean("CMDTUALRng", false);
            SmartDashboard.putBoolean("CMDTI2SWLRng", false);

            // Front Camera

            ShuffleboardTab cameraTab = Shuffleboard.getTab("Vision");

            cameraTab.add("No Zoom", new InstantCommand(() -> m_limelight.setPipeline(0)));
            cameraTab.add("2XZoom", new InstantCommand(() -> m_limelight.setPipeline(1)));
            cameraTab.add("3X Zoom", new InstantCommand(() -> m_limelight.setPipeline(2)));
            cameraTab.add("Camtran", new InstantCommand(() -> m_limelight.setPipeline(9)));
            cameraTab.add("DriverCam",
                        new InstantCommand(() -> m_limelight.setCamMode(LimelightControlMode.CamMode.kdriver)));
            cameraTab.add("TargetCam",
                        new InstantCommand(() -> m_limelight.setCamMode(LimelightControlMode.CamMode.kvision)));
            cameraTab.add("LEDsOn",
                        new InstantCommand(() -> m_limelight.setLEDMode(LimelightControlMode.LedMode.kforceOn)));
            cameraTab.add("LEDsOff",
                        new InstantCommand(() -> m_limelight.setLEDMode(LimelightControlMode.LedMode.kforceOff)));
            cameraTab.add("LEDsPipe",
                        new InstantCommand(() -> m_limelight.setLEDMode(LimelightControlMode.LedMode.kpipeLine)));

            // HttpCamera httpCamera = new HttpCamera("LimelightCamera",
            // "http://frcvision.local:1181/stream.mjpg");
            // CameraServer.getInstance().addCamera(httpCamera);
            // Shuffleboard.getTab("cameraTab").add(httpCamera);

            // Configure default commands
            // Set the default drive command to joystick arcade drive

            // A joystick arcade command, with forward/backward controlled by the left
            // hand, and turning controlled by the twist.

            m_robotDrive.setDefaultCommand(
                        // A joystick arcade command, with forward/backward controlled by the left
                        // hand, and turning controlled by the twist.
                        new RunCommand(() -> m_robotDrive.arcadeDrive(-m_driverController.getY(),
                                    m_driverController.getTwist() / 3), m_robotDrive));

            m_controlPanel.setDefaultCommand(new RunCommand(
                        () -> m_controlPanel.turnWheelMotor(setupGamepad.getY(Hand.kLeft) / 5), m_controlPanel));

            m_shooter.setDefaultCommand(
                        new RunCommand(() -> m_shooter.jogShooter(gamepad.getX(Hand.kLeft) / 5), m_shooter));

            m_turret.setDefaultCommand(new PositionTurret(m_turret));

            m_tilt.setDefaultCommand(new PositionTilt(m_tilt));

            new RunCommand(() -> m_transport.runLeftBeltMotor(setupGamepad.getX(Hand.kLeft) / 2), m_transport);
      }

      /**
       * Use this method to define your button->command mappings. Buttons can be
       * created by instantiating a {@link GenericHID} or one of its subclasses
       * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
       * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
       */
      private void configureButtonBindings() {

            // Driver Joystick

            new JoystickButton(m_driverController, 1).whileHeld(new StartRearIntake(m_rearIntake));

            new JoystickButton(m_driverController, 2)
                        .whenPressed(new ShootCells(m_shooter, m_transport, m_compressor, 3000, 0));

            new JoystickButton(m_driverController, 3).whenPressed(new StopShoot(m_shooter, m_transport));

            new JoystickButton(m_driverController, 4).whenPressed(new SetCameraPipeline(m_limelight, 0));// normal zoom

            new JoystickButton(m_driverController, 5).whenPressed(new SetCameraPipeline(m_limelight, 1)); // 2 x zoom

            new JoystickButton(m_driverController, 6).whenPressed(new ToggleDriverCam(m_limelight));

            // co driver gamepad
            new JoystickButton(gamepad, Button.kY.value)
                        .whenPressed(new InstantCommand(m_shooter::increaseShooterSpeed));

            new JoystickButton(gamepad, Button.kA.value)
                        .whenPressed(new InstantCommand(m_shooter::decreaseShooterSpeed));

            new JoystickButton(gamepad, Button.kX.value)
                        .whenPressed(new InstantCommand(() -> m_turret.changeTurretOffset(false)));

            new JoystickButton(gamepad, Button.kB.value)
                        .whenPressed(new InstantCommand(() -> m_turret.changeTurretOffset(true)));

            // new JoystickButton(gamepad, Button.kStickLeft.value)

            // new JoystickButton(gamepad, Button.kStickRight.value)

            // new JoystickButton(gamepad, Button.kBumperRight.value)

            // new JoystickButton(gamepad, Button.kStart.value)

            // new JoystickButton(gamepad, Button.kBack.value)

            // Setup Gamepad

            new JoystickButton(setupGamepad, Button.kA.value)
                        .whenPressed(() -> m_rearIntake.runIntakeMotor(IntakeConstants.REAR_SPEED))
                        .whenPressed(() -> m_transport.runFrontRollerMotor(CellTransportConstants.FRONT_SHOOT_SPEED))
                        .whenPressed(() -> m_transport.runRearRollerMotor(-CellTransportConstants.REAR_SHOOT_SPEED))
                        .whenPressed(() -> m_transport.runLeftBeltMotor(CellTransportConstants.BELT_SPEED))
                        .whenReleased(() -> m_rearIntake.runIntakeMotor(0.))
                        .whenReleased(() -> m_transport.runFrontRollerMotor(0.))
                        .whenReleased(() -> m_transport.runRearRollerMotor(0.))
                        .whenReleased(() -> m_transport.runLeftBeltMotor(0.));

            new JoystickButton(setupGamepad, Button.kBumperLeft.value).whileHeld(new StartEndCommand(
                        () -> m_shooter.jogShooter(.05), () -> m_shooter.jogShooter(0.), m_shooter));
            new JoystickButton(setupGamepad, Button.kBumperRight.value).whileHeld(new StartEndCommand(
                        () -> m_shooter.jogShooter(-.05), () -> m_shooter.jogShooter(0.), m_shooter));

            new JoystickButton(setupGamepad, Button.kX.value)
                        .whenPressed(new RunCommand(() -> m_turret.jogTurret(setupGamepad.getX(Hand.kRight) / 2),
                                    m_turret))
                        .whenPressed(
                                    new RunCommand(() -> m_tilt.jogTilt(-setupGamepad.getY(Hand.kRight) / 10), m_tilt));

            new JoystickButton(setupGamepad, Button.kB.value).whenPressed(new StartShooter(m_shooter));

            CommandScheduler.getInstance()
                        .onCommandInitialize(command -> System.out.println(command.getName() + " is starting"));
            CommandScheduler.getInstance()
                        .onCommandFinish(command -> System.out.println(command.getName() + " has ended"));
            CommandScheduler.getInstance()
                        .onCommandInterrupt(command -> System.out.println(command.getName() + " was interrupted"));
            CommandScheduler.getInstance().onCommandInitialize(
                        command -> SmartDashboard.putString("CS", command.getName() + " is starting"));
            CommandScheduler.getInstance()
                        .onCommandFinish(command -> SmartDashboard.putString("CE", command.getName() + " has Ended"));
            CommandScheduler.getInstance().onCommandInterrupt(
                        command -> SmartDashboard.putString("CE", command.getName() + "was Interrupted"));

            LiveWindow.disableAllTelemetry();

      }

      /**
       * Use this to pass the autonomous command to the main {@link Robot} class.
       *
       * @return the command to run in autonomous
       */

      public Command getAutonomousCommand() {
            return null;

      }

}
