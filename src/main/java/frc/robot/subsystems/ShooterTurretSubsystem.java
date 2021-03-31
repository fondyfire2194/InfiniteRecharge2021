/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodedShooterConstants;

/**
 * On power up turret will reference to 0 position In competition, turret will
 * be placed in a straight ahead position initially. Turret can be jogged using
 * TalonSRX PercentOut mode. Turret can be postioned using TalonSRX MotionMagic
 * Mode
 * 
 * In autonomous, turret will be positioned to an angle from a table based on
 * field starting position. Once target is seen and targeting information comes
 * from camera, the distance and angle error will be calculated and added to the
 * MotionMagic endpoint
 * 
 * 
 * 
 */
public class ShooterTurretSubsystem extends SubsystemBase {
   /**
    * Creates a new ShooterTurret.
    */

   public WPI_TalonSRX m_rotateMotor = new WPI_TalonSRX(HoodedShooterConstants.ROTATE_MOTOR);

   public String kEnable;
   public String kDisable;
   public double requiredTurretAngle;

   public double targetHorizontalOffset;
   public double commandAngle;
   private int displaySelect;
   public boolean lockOnTarget;
   public boolean changeLocked;
   private double encoderCountsPerRev = 4096;
   private double degreesPerEncoderRev = .00295; // HoodedShooterConstants.TURRET_ENCODER_DEG_PER_REV
   public double encoderCountsPerDegree = encoderCountsPerRev / degreesPerEncoderRev; // 4096 / .00295 = 13884.458
   public double positionCommandAngle;
   public double holdPositionDegrees;

   private static final int TALON_TIMEOUT_MS = 20;
   public static final int TICKS_PER_REVOLUTION = 2048; //

   static final int TALON_TICK_THRESH = 128;
   static final double TICK_THRESH = TALON_TICK_THRESH * 4;
   public static final double TICK_PER_100MS_THRESH = 64; // about a tenth of a rotation per second
   static final int PRIMARY_PID_LOOP = 0;
   public double visionCorrection;

   public ShooterTurretSubsystem() {
      m_rotateMotor.set(ControlMode.PercentOutput, 0);
      m_rotateMotor.configFactoryDefault();
      m_rotateMotor.setNeutralMode(NeutralMode.Brake);
      m_rotateMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PRIMARY_PID_LOOP, TALON_TIMEOUT_MS);
      m_rotateMotor.setSensorPhase(true);
      m_rotateMotor.configVoltageCompSaturation(12, 0);
      m_rotateMotor.enableVoltageCompensation(true);
      m_rotateMotor.configAllowableClosedloopError(5000, TALON_TICK_THRESH, TALON_TIMEOUT_MS);
      m_rotateMotor.configForwardSoftLimitEnable(false);
      m_rotateMotor.configReverseSoftLimitEnable(false);
      m_rotateMotor.configForwardSoftLimitThreshold(110);
      m_rotateMotor.configReverseSoftLimitThreshold(100);
      m_rotateMotor.configMotionSCurveStrength(2);
      /* Set acceleration and vcruise velocity - see documentation */
      final int cruiseVelocitydegreesPerSec = 10;
      int cruiseVelocityCountsPer100ms = cruiseVelocitydegreesPerSec
            * HoodedShooterConstants.TURRET_ENCODER_COUNTS_PER_100MS_PER_TURRET_DEGREES_PER_SEC;
      m_rotateMotor.configMotionCruiseVelocity(cruiseVelocityCountsPer100ms, TALON_TIMEOUT_MS);
      m_rotateMotor.configMotionAcceleration(cruiseVelocityCountsPer100ms * 3, TALON_TIMEOUT_MS);

      resetTurretPosition();
      commandAngle = 0;
      targetHorizontalOffset = 0;
      lockOnTarget = true;
      SmartDashboard.putString("TurretState", "Init");

   }

   @Override
   public void periodic() {
      // This method will be called once per scheduler run
      displaySelect++;
      if (displaySelect >= 23) {
         displaySelect = 0;
         SmartDashboard.putNumber("Turret Angle", getTurretAngle());
         SmartDashboard.putNumber("Turret Amps", m_rotateMotor.getStatorCurrent());

      }
   }

   public int getTurretEncoderPosition() {
      return m_rotateMotor.getSelectedSensorPosition(0);
   }

   public void resetTurretPosition() {
      m_rotateMotor.setSelectedSensorPosition(0, 0, TALON_TIMEOUT_MS);
      commandAngle = 0;
   }

   public int getTurretEncoderSpeedCountsPer100mS() {
      return m_rotateMotor.getSelectedSensorVelocity(0);
   }

   public double getTurretAngle() {
      return m_rotateMotor.getSelectedSensorPosition(0) / encoderCountsPerDegree;
   }

   public void jogTurret(double speed) {
      m_rotateMotor.set(ControlMode.PercentOutput, speed);

   }

   public void motionMagic(double angle) {
      double correctedAngle = angle + visionCorrection;
      correctedAngle *= encoderCountsPerDegree;
      m_rotateMotor.set(ControlMode.MotionMagic, correctedAngle);
   }

   public double getTurretOut() {
      return m_rotateMotor.get();
   }

   public boolean turretInPosition() {

      return Math.abs(commandAngle - getTurretAngle()) < 2;

   }

   public void changeTurretOffset(boolean right) {
      if (right)
         targetHorizontalOffset += .25;
      else
         targetHorizontalOffset -= .25;
   }

   public double iterateTurretPosition(double speed) {

      return commandAngle + speed / 100;
   }

}
