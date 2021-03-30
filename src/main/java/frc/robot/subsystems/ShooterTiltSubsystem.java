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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodedShooterConstants;

public class ShooterTiltSubsystem extends SubsystemBase {
   /**
    * Creates a new ShooterTurret.
    */

   public final WPI_TalonSRX m_tiltMotor = new WPI_TalonSRX(HoodedShooterConstants.TILT_MOTOR);
   static final int TALON_TICK_THRESH = 128;
   static final double TICK_THRESH = TALON_TICK_THRESH * 4;
   static final int PRIMARY_PID_LOOP = 0;
   private static final int TALON_TIMEOUT_MS = 20;
   private double encoderCountsPerRev = 4096;
   private double degreesPerEncoderRev = .00295; // HoodedShooterConstants.TURRET_ENCODER_DEG_PER_REV
   private double encoderCountsPerDegree = encoderCountsPerRev * degreesPerEncoderRev; // 4096 * .00295 = 12.0832
 
   public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

   public double requiredTiltAngle;

   // public double commandTurns;
   public double targetVerticalOffset;
   private int displaySelect;
   public boolean loookForTarget;
   public boolean changeLocked;

   public double positionCommandTurns;

   public boolean positionResetDone;

   private boolean switchPositionLast;

   public double lastHoldPositionDegrees;
   public double holdPositionDegrees;

   public DigitalInput m_reverseLimit = new DigitalInput(6);
   private double tiltTargetPosition;
   public double m_tiltVisionCorrection;

   public ShooterTiltSubsystem() {
      m_tiltMotor.set(ControlMode.PercentOutput, 0);
      m_tiltMotor.configFactoryDefault();
      m_tiltMotor.setNeutralMode(NeutralMode.Brake);
      m_tiltMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PRIMARY_PID_LOOP, TALON_TIMEOUT_MS);
      m_tiltMotor.setSensorPhase(true);
      m_tiltMotor.configVoltageCompSaturation(12, 0);
      m_tiltMotor.enableVoltageCompensation(true);
      m_tiltMotor.configAllowableClosedloopError(0, TALON_TICK_THRESH, TALON_TIMEOUT_MS);
      m_tiltMotor.configForwardSoftLimitEnable(false);
      m_tiltMotor.configReverseSoftLimitEnable(false);
      m_tiltMotor.configForwardSoftLimitThreshold(110);
      m_tiltMotor.configReverseSoftLimitThreshold(10);
      m_tiltMotor.configMotionSCurveStrength(2);
      /* Set acceleration and vcruise velocity - see documentation */
      m_tiltMotor.configMotionCruiseVelocity(3000, TALON_TIMEOUT_MS);
      m_tiltMotor.configMotionAcceleration(3000, TALON_TIMEOUT_MS);

      // setPIDParameters();
      positionResetDone = false;
      targetVerticalOffset = 0;

      if (m_reverseLimit.get()) {
         resetTiltPosition();
         positionCommandTurns = 0;
         positionResetDone = true;
      }
      SmartDashboard.putString("TiltState", "Init");
   }

   public int getTltEncoderSpeedCountsPer100mS() {
      return m_tiltMotor.getSelectedSensorVelocity(0);
   }

   public int getTiltEncoderPosition() {
      return m_tiltMotor.getSelectedSensorPosition(0);
   }

   public double getTiltPositionDegrees() {
      return getTiltEncoderPosition()/encoderCountsPerDegree;
   }

   public boolean inPosition() {
      return Math.abs(tiltTargetPosition - getTiltPositionDegrees()) < 2;
   }

   @Override
   public void periodic() {
      // This method will be called once per scheduler run

      if (m_reverseLimit.get() && (!positionResetDone || !switchPositionLast)) {
         resetTiltPosition();
         positionCommandTurns = 0;
         positionResetDone = true;

      }

      displaySelect++;
      if (displaySelect >= 7) {
         displaySelect = 0;

         SmartDashboard.putNumber("TiltPosn", getTiltPositionDegrees());
         SmartDashboard.putNumber("Tilt Angle", getTiltAngle());
         SmartDashboard.putNumber("Tilt OUT", getTiltOut());
         SmartDashboard.putNumber("Tilt Amps", m_tiltMotor.getStatorCurrent());
         SmartDashboard.putBoolean("Tilt Down R LS", m_reverseLimit.get());
         SmartDashboard.putNumber("TiltComTrns", positionCommandTurns);
         SmartDashboard.putBoolean("TILT DOWN LIMITS", positionResetDone);

      }
   }

   public void resetTiltPosition() {

      m_tiltMotor.setSelectedSensorPosition(0, 0, TALON_TIMEOUT_MS);
   }

   public void jogTilt(double speed) {
      /
      SmartDashboard.putString("TiltState", "Jogging");

      m_tiltMotor.set(speed);
   }

   public void motionMagic(double position) {
      m_tiltMotor.set(ControlMode.MotionMagic, position);

   }

   public double getTiltOut() {
      return m_tiltMotor.get();
   }

   public double getTiltAngle() {
      return 0;// m_tiltEncoder.getDistance();
   }

   public boolean getTiltInPosition() {
      return true;
   }

   public double getTiltSpeed() {
      return getTltEncoderSpeedCountsPer100mS() * 10;
   }

   public void runTiltMotor(double speed) {
      m_tiltMotor.set(ControlMode.Velocity, speed);
   }

}
