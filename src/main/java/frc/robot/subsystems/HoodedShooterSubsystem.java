/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodedShooterConstants;

public class HoodedShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new HoodedShooter.
   */
  public CANSparkMax m_leftMotor = new CANSparkMax(HoodedShooterConstants.LEFT_MOTOR, MotorType.kBrushless);
  public CANSparkMax m_rightMotor = new CANSparkMax(HoodedShooterConstants.RIGHT_MOTOR, MotorType.kBrushless);
  private final CANEncoder m_leftEncoder = m_leftMotor.getEncoder();
  private final CANEncoder m_rightEncoder = m_rightMotor.getEncoder();
  private final CANPIDController m_rightPIDController = m_rightMotor.getPIDController();

  public List<Integer> sparkMaxID;
  public List<CANSparkMax> motor;

  public int currentSpeedIndex = 0;
  public double requiredSpeed = 1500;
  private int displaySelect;
  public double requiredSpeedLast;
  public boolean isShooting;
  public double shootTime;
  public double distanceCalculated;

  private double[] distance = { .03, 1.4, 2.4, 3.4, 4.1, 4.3, 4.9, 5.3, 5.6, 5.7, 5.8, 5.9, 6.0, 6.1, 6.2 };
  private double baseDistance = 5.3;

  public HoodedShooterSubsystem() {
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();

    // motor.add(m_leftMotor);
    // motor.add(m_rightMotor);
    // sparkMaxID.add(m_leftMotor.getDeviceId());
    // sparkMaxID.add(m_rightMotor.getDeviceId());

    // m_rightMotor.setControlFramePeriodMs(10);
    // m_leftMotor.setControlFramePeriodMs(10);

    // m_leftMotor.setClosedLoopRampRate(0.5);
    m_rightMotor.setClosedLoopRampRate(.5);

    // m_leftMotor.enableVoltageCompensation(10);
    // m_rightMotor.enableVoltageCompensation(10);

    m_leftMotor.follow(m_rightMotor, true);
    setBrakesOn(true);
    refreshGains();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    displaySelect++;
    if (displaySelect > 2) {
      displaySelect = 0;
      SmartDashboard.putNumber("ShooterLeftPosition", getLeftEncoderRevs());
      SmartDashboard.putNumber("ShooterRightPosition", getRightEncoderRevs());
      SmartDashboard.putNumber("ShooterLeftRPM", getLeftEncoderRPM());
      SmartDashboard.putNumber("ShooterRightRPM", getRightEncoderRPM());

      SmartDashboard.putNumber("ShooterRightAmps", m_rightMotor.getOutputCurrent());
      SmartDashboard.putNumber("ShooterLeftAmps", m_leftMotor.getOutputCurrent());
      SmartDashboard.putNumber("ShooterRightTemp", m_rightMotor.getMotorTemperature());
      SmartDashboard.putNumber("ShooterLeftTemp", m_leftMotor.getMotorTemperature());
      SmartDashboard.putBoolean("ShooterisFollower", m_rightMotor.isFollower());
      SmartDashboard.putNumber("ShooterRqdRPM", requiredSpeed);
      SmartDashboard.putBoolean("AtShootSpeed", getAtSpeed());
      SmartDashboard.putNumber("VisionDistance", distanceCalculated);

    }
  }

  public boolean getAtSpeed() {
    return Math.abs(requiredSpeed - getLeftEncoderRPM()) < 250.;
  }

  public double getLeftEncoderRevs() {
    return m_leftEncoder.getPosition();
  }

  public double getRightEncoderRevs() {
    return m_rightEncoder.getPosition();
  }

  public double getLeftEncoderRPM() {
    return m_leftEncoder.getVelocity();
  }

  public double getRightEncoderRPM() {
    return m_rightEncoder.getVelocity();
  }

  public void jogShooter(double speed) {
    // m_leftMotor.set(-speed);
    m_rightMotor.set(speed);
  }

  public void increaseShooterSpeed() {
    requiredSpeed += HoodedShooterConstants.SPEED_INCREMENT;
    if (requiredSpeed > HoodedShooterConstants.MAX_SPEED) {
      requiredSpeed = HoodedShooterConstants.MAX_SPEED;
    }
  }

  public void decreaseShooterSpeed() {
    requiredSpeed -= HoodedShooterConstants.SPEED_INCREMENT;
    if (requiredSpeed < HoodedShooterConstants.MIN_SPEED) {
      requiredSpeed = HoodedShooterConstants.MIN_SPEED;
    }
  }

  public void runShooterVelMode() {
    // m_leftPIDController.setReference(-requiredSpeed, ControlType.kVelocity);
    if (requiredSpeedLast != requiredSpeed) {
      m_rightPIDController.setReference(requiredSpeed, ControlType.kVelocity);
      requiredSpeedLast = requiredSpeed;
    }
  }

  public void runShooterVelMode(double rpm) {
    if (rpm > HoodedShooterConstants.MAX_SPEED - 1)
      rpm = HoodedShooterConstants.MAX_SPEED;
    if (rpm < HoodedShooterConstants.MIN_SPEED)
      rpm = HoodedShooterConstants.MIN_SPEED;
    m_rightPIDController.setReference(rpm, ControlType.kVelocity);
  }

  public double findDistance(double boxHeight) {

    if (boxHeight > 70)
      boxHeight = 70;
    if (boxHeight < 0)
      boxHeight = 0;
    double i = 0;
    double m = 0;
    double j = boxHeight / 10;
    double k = boxHeight % 10;
    if (k >= 5) {
      i = 5;
      m = k - 5;
    } else {
      i = 0;
      m = k;
    }
    double l = (j * 10 + i) - 30;
    int n = (int) l / 5;
    double d1 = distance[n];
    double d2 = distance[n + 1];
    double d3 = (d2 - d1);
    double d4 = (d3 * m / 5);
    double d5 = d1 + d4;

    return baseDistance - d5;
  }

  public void stopShooterVeLMode() {
    // m_leftPIDController.setReference(0., ControlType.kVelocity);

    m_rightPIDController.setReference(0., ControlType.kVelocity);
    requiredSpeedLast = requiredSpeed;
  }

  public boolean detectCellFired() {
    return m_leftMotor.getOutputCurrent() > 10.;
  }

  public void setBrakesOn(boolean on) {
    if (on) {
      m_leftMotor.setIdleMode(IdleMode.kBrake);
      m_rightMotor.setIdleMode(IdleMode.kBrake);
    } else {
      m_leftMotor.setIdleMode(IdleMode.kCoast);
      m_rightMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public void refreshGains() {
    // PID coefficients
    // kP = 6e-4;
    // kI = 0;
    // kD = 0;
    // kIz = 0;
    // kFF = 0.00015;
    // kMaxOutput = 1;
    // kMinOutput = -1;
    // maxRPM = 5700;
    m_rightPIDController.setP(3e-4);
    m_rightPIDController.setI(0.00000);
    m_rightPIDController.setD(0.);
    m_rightPIDController.setIZone(1000.);
    m_rightPIDController.setFF(.00017);
    m_rightPIDController.setOutputRange(-1., 1.);

  }

}
