/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.SimpleCSVLogger;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends SubsystemBase {
   public CANSparkMax m_leftMaster = new CANSparkMax(DriveConstants.DRIVETRAIN_LEFT_MASTER, MotorType.kBrushless);
   public CANSparkMax m_leftFollower = new CANSparkMax(DriveConstants.DRIVETRAIN_LEFT_FOLLOWER, MotorType.kBrushless);
   public CANSparkMax m_rightMaster = new CANSparkMax(DriveConstants.DRIVETRAIN_RIGHT_MASTER, MotorType.kBrushless);
   public CANSparkMax m_rightFollower = new CANSparkMax(DriveConstants.DRIVETRAIN_RIGHT_FOLLOWER, MotorType.kBrushless);

   private final CANEncoder m_leftMaster_encoder = m_leftMaster.getEncoder();
   private final CANEncoder m_leftFollower_encoder = m_leftFollower.getEncoder();
   private final CANEncoder m_rightMaster_encoder = m_rightMaster.getEncoder();
   private final CANEncoder m_rightFollower_encoder = m_rightFollower.getEncoder();

   private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

   private final DifferentialDriveOdometry m_odometry;

   public static AHRS imu;
   private int displaySelect;

   public double[] distanceArray2x = new double[50];

   // private final TimeOfFlight m_rangeSensor = new TimeOfFlight(10);

   //
   ShuffleboardTab driveTab;

   private double kTrackWidth = DriveConstants.WHEELBASE_WIDTH;
   private final PIDController m_leftPIDController = new PIDController(1.45, 0, 0);
   private final PIDController m_rightPIDController = new PIDController(1.45, 0, 0);
   // private final DifferentialDriveKinematics m_kinematics = new
   // DifferentialDriveKinematics(kTrackWidth);

   public SimpleCSVLogger simpleCSVLogger;
   public boolean endFile;
   public boolean endTrajFile;
   public List<Integer> sparkMaxID;
   public List<CANSparkMax> motor;

   // Put methods for controlling this subsystem
   // here. Call these from Commands.
   public DriveSubsystem() {

      CANError lfm = m_leftMaster.restoreFactoryDefaults();
      SmartDashboard.putBoolean("LFM", lfm == CANError.kOk);
      CANError lff = m_leftFollower.restoreFactoryDefaults();
      SmartDashboard.putBoolean("LFF", lff == CANError.kOk);
      CANError rfm = m_rightMaster.restoreFactoryDefaults();
      SmartDashboard.putBoolean("RFM", rfm == CANError.kOk);
      CANError rff = m_rightFollower.restoreFactoryDefaults();
      SmartDashboard.putBoolean("RFF", rff == CANError.kOk);
      m_leftFollower.follow(m_leftMaster);
      m_rightFollower.follow(m_rightMaster);

      // motor.add(m_leftMaster);
      // motor.add(m_leftFollower);
      // motor.add(m_rightMaster);
      // motor.add(m_rightFollower);

      // sparkMaxID.add(m_leftMaster.getDeviceId());
      // sparkMaxID.add(m_leftFollower.getDeviceId());
      // sparkMaxID.add(m_rightMaster.getDeviceId());
      // sparkMaxID.add(m_rightFollower.getDeviceId());

      setLeftBrakeOn(false);
      setRightBrakeOn(false);
      simpleCSVLogger = new SimpleCSVLogger();

      try {
         // imu = new AHRS(I2C.Port.kOnboard);

         // imu = new AHRS(SPI.Port.kMXP);
         imu = new AHRS(SerialPort.Port.kUSB1);

      } catch (Exception ex) {
         DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
      }

      SmartDashboard.putString("NavX FW#", imu.toString());

      m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
      // m_rangeSensor.setRangingMode(RangingMode.Short, 40);

   }

   @Override
   public void periodic() {

      // This method will be called once per scheduler run

      updateOdometry();

      displaySelect++;
      if (displaySelect >= 25) {
         displaySelect = 0;

         SmartDashboard.putNumber("LeftM", getLeftDistanceMeters());
         SmartDashboard.putNumber("RightM", getRightDistanceMeters());

         SmartDashboard.putNumber("LeftEnc", m_leftMaster_encoder.getPosition());
         SmartDashboard.putNumber("RightEnc", m_rightMaster_encoder.getPosition());
         SmartDashboard.putNumber("Lmpersec", getLeftEncoderMps());
         SmartDashboard.putNumber("Rmpersec", getRightEncoderMps());

         SmartDashboard.putNumber("LMAmps", m_leftMaster.getOutputCurrent());
         SmartDashboard.putNumber("RMAmps", m_rightMaster.getOutputCurrent());
         SmartDashboard.putNumber("LFAmps", m_leftFollower.getOutputCurrent());
         SmartDashboard.putNumber("RFAmps", m_rightFollower.getOutputCurrent());

         SmartDashboard.putNumber("GyroYaw", getGyroYaw());
         SmartDashboard.putNumber("GyroHeading", getHeading());
         SmartDashboard.putNumber("XPosn", getX());
         SmartDashboard.putNumber("YPosn", getY());
         SmartDashboard.putString("Pose", getPose().toString());

         SmartDashboard.putNumber("LLLMO", m_leftMaster.get());
         // SmartDashboard.putNumber("LLLFO", m_leftFollower.get());
         SmartDashboard.putNumber("RRRMO", m_rightMaster.get());
         // SmartDashboard.putNumber("RRRFO", m_rightFollower.get());
         // SmartDashboard.putNumber("TOFRange MM", m_rangeSensor.getRange());
         // SmartDashboard.putNumber("TOFStdDev MM", m_rangeSensor.getRangeSigma());
         // SmartDashboard.putString("TOFRange MM",
         // m_rangeSensor.getStatus().toString());

      }
   }

   /*
    * Drives the robot using arcade controls
    *
    * @param fwd the commanded forward movement
    * 
    * @param rot the commanded rotation
    */
   public void arcadeDrive(double fwd, double rot) {
      if (Math.abs(fwd) < .1)
         fwd = 0;
      if (Math.abs(rot) < .1)
         rot = 0;
      m_drive.arcadeDrive(fwd, rot);
   }

   public void tankDrive(double left, double right) {
      m_drive.tankDrive(left, right);
   }

   /**
    * Drives the robot with the given linear velocity and angular velocity.
    *
    * @param xSpeed Linear velocity in m/s.
    * @param rot    Angular velocity in rad/s.
    */
   @SuppressWarnings("ParameterName")
   public void drive(double xSpeed, double rot) {
      var wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
      setSpeeds(wheelSpeeds);
   }

   /**
    * Sets the desired wheel speeds.
    *
    * @param speeds The desired wheel speeds.
    */
   public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
      SmartDashboard.putNumber("LeftSide", speeds.leftMetersPerSecond);
      SmartDashboard.putNumber("RightSide", speeds.rightMetersPerSecond);
      double leftOutput = m_leftPIDController.calculate(getLeftEncoderMps(), speeds.leftMetersPerSecond);
      double rightOutput = m_rightPIDController.calculate(getRightEncoderMps(), speeds.rightMetersPerSecond);
      m_leftMaster.set(leftOutput);
      m_rightMaster.set(rightOutput);
   }

   /**
    * Returns the angle of the robot as a Rotation2d.
    *
    * @return The angle of the robot.
    */
   public double getHeading() {
      return Math.IEEEremainder(imu.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);

   }

   /**
    * Returns the angle of the robot as a Rotation2d.
    *
    * @return The angle of the robot.
    */
   public Rotation2d getAngle() {
      return Rotation2d.fromDegrees(getHeading());
   }

   /**
    * Updates the field-relative position.
    */
   public void updateOdometry() {
      var gyroAngle = getAngle();
      m_odometry.update(gyroAngle, getLeftDistanceMeters(), getRightDistanceMeters());

   }

   public double getGyroYaw() {
      return imu.getYaw();
   }

   public double getGyroAngle() {
      return -imu.getAngle();

   }

   public void resetOdometry(Pose2d pose) {
      m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
   }

   public void resetGyro() {
      imu.reset();
   }

   public void resetPose(Rotation2d angle) {
      resetEncoders();
      resetPose(angle);
   }

   public void resetAll() {
      resetGyro();
      resetEncoders();
   }

   public Pose2d getPose() {
      return m_odometry.getPoseMeters();
   }

   public Translation2d getTranslation() {
      return getPose().getTranslation();
   }

   public double getX() {
      return getTranslation().getX();
   }

   public double getY() {
      return getTranslation().getY();
   }

   public double getLeftDistanceMeters() {
      return m_leftMaster_encoder.getPosition() * DriveConstants.METERS_PER_MOTOR_REV;
   }

   public double getRightDistanceMeters() {
      return -m_rightMaster_encoder.getPosition() * DriveConstants.METERS_PER_MOTOR_REV;
   }

   public double getAverageDistanceMeters() {
      return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2;
   }

   public double getLeftEncoderMps() {
      return m_leftMaster_encoder.getVelocity() * DriveConstants.METERS_PER_MOTOR_REV / 60;
   }

   public double getRightEncoderMps() {
      return -m_rightMaster_encoder.getVelocity() * DriveConstants.METERS_PER_MOTOR_REV / 60;
   }

   public void setEncoderPosition(double position) {
      m_leftMaster_encoder.setPosition(position * DriveConstants.METERS_PER_MOTOR_REV);
      m_leftFollower_encoder.setPosition(position * DriveConstants.METERS_PER_MOTOR_REV);
      m_rightMaster_encoder.setPosition(position * DriveConstants.METERS_PER_MOTOR_REV);
      m_rightFollower_encoder.setPosition(position * DriveConstants.METERS_PER_MOTOR_REV);
   }

   public void resetEncoders() {
      setEncoderPosition(0.);
   }

   private void setLeftBrakeOn(boolean on) {
      if (on) {
         m_leftMaster.setIdleMode(IdleMode.kBrake);
         m_leftFollower.setIdleMode(IdleMode.kBrake);
      } else {
         m_leftMaster.setIdleMode(IdleMode.kCoast);
         m_leftFollower.setIdleMode(IdleMode.kCoast);
      }
   }

   private void setRightBrakeOn(boolean on) {
      if (on) {
         m_rightMaster.setIdleMode(IdleMode.kBrake);
         m_rightFollower.setIdleMode(IdleMode.kBrake);
      } else {
         m_rightMaster.setIdleMode(IdleMode.kCoast);
         m_rightFollower.setIdleMode(IdleMode.kCoast);
      }
   }

   /**
    * Controls the left and right sides of the drive directly with voltages.
    *
    * @param leftVolts  the commanded left output
    * @param rightVolts the commanded right output
    */
   public void tankDriveVolts(double leftVolts, double rightVolts) {
      if (!Robot.isSimulation()) {
         m_leftMaster.setVoltage(leftVolts);
         m_rightMaster.setVoltage(-rightVolts);
         m_drive.feed();
      } else {
         m_leftMaster.set(leftVolts);
         m_rightMaster.set(-rightVolts);
         m_drive.feed();

      }
   }

   /**
    * Returns the current wheel speeds of the robot.
    *
    * @return The current wheel speeds.
    */
   public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds(getLeftEncoderMps(), getRightEncoderMps());
   }
}
