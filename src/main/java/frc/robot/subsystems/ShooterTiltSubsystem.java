/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.Pref;

public class ShooterTiltSubsystem extends SubsystemBase {
   /**
    * Creates a new ShooterTurret.
    */

   public final WPI_TalonSRX m_tiltMotor = new WPI_TalonSRX(HoodedShooterConstants.TILT_MOTOR);

   private final Encoder m_tiltEncoder = new Encoder(1, 2);

   private final double k_encoderCountsPerDegree = 1 / HoodedShooterConstants.TILT_DEG_PER_ENCODER_REV;

   private final PIDController tiltPositionController = new PIDController(.05, 0.01, 0);
   private final PIDController tiltLockController = new PIDController(.032, 0.001, 0);
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

   public ShooterTiltSubsystem() {
      m_tiltEncoder.setDistancePerPulse(k_encoderCountsPerDegree);
      positionResetDone = false;
      targetVerticalOffset = 0;

      setTiltPosGains();
      setTiltLockGains();

      if (m_reverseLimit.get()) {
         resetTiltPosition();
         positionCommandTurns = 0;
         positionResetDone = true;

      }

      SmartDashboard.putString("TiltState", "Init");

   }

   public double getTiltSpeedCountsPer100mS() {
      return m_tiltEncoder.getRate() * 10;
   }

   public double getTiltPositionDegrees() {
      return m_tiltEncoder.getDistance();
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
      m_tiltEncoder.reset();
      positionCommandTurns = 0;
   }

   public void jogTilt(double speed) {
      positionCommandTurns = m_tiltEncoder.getDistance();
      SmartDashboard.putString("TiltState", "Jogging");

      m_tiltMotor.set(speed);
   }

   public void positionTilttoTurns(double turns) {
      positionCommandTurns = turns;
      positionTilttoTurns();
   }

   public void positionTilttoTurns() {

      double pidOut = tiltPositionController.calculate(getTiltPositionDegrees(), positionCommandTurns);
      m_tiltMotor.set(pidOut);
      SmartDashboard.putNumber("PIDORRTilt", pidOut);
      // positionCommandTurns = getTiltPosition();
      SmartDashboard.putNumber("TIPosErr", tiltPositionController.getPositionError());
      SmartDashboard.putString("TiltState", "Positioning");

   }

   public double getTiltOut() {
      return m_tiltMotor.get();
   }

   public double getTiltAngle() {
      return m_tiltEncoder.getDistance();
   };

   public boolean getTiltInPosition() {
      return true;
   }

   public double getTiltSpeed() {
      return m_tiltEncoder.getRate();
   }

   public void runTiltMotor(double speed) {
      m_tiltMotor.set(ControlMode.Velocity, speed);
   }

   public boolean lockTiltToVision(double cameraError) {
      double pidOut = tiltLockController.calculate(cameraError, 0);
      m_tiltMotor.set(pidOut);
      SmartDashboard.putNumber("PIDOTilt", pidOut);
      SmartDashboard.putNumber("TILockErr", tiltLockController.getPositionError());
      positionCommandTurns = getTiltPositionDegrees();

      SmartDashboard.putString("TiltState", "VisionLock");

      return tiltLockController.atSetpoint();
   }

   public void changeTiltOffset(boolean up) {
      if (up)
         targetVerticalOffset += .25;
      else
         targetVerticalOffset -= .25;
   }

   private void setTiltPosGains() {

      tiltPositionController.setP(Pref.getPref("TiPkP"));
      tiltPositionController.setI(Pref.getPref("TiPkI"));
      tiltPositionController.setD(Pref.getPref("TiPkD"));
      double Izone = Pref.getPref("TiPkIZ");
      tiltPositionController.setIntegratorRange(-Izone, Izone);
      tiltPositionController.setTolerance(.5);
   }

   private void setTiltLockGains() {

      tiltLockController.setP(Pref.getPref("TiLkP"));
      tiltLockController.setI(Pref.getPref("TiLkI"));
      tiltLockController.setD(Pref.getPref("TiLkD"));
      double Izone = Pref.getPref("TiLkIZ");
      tiltLockController.setIntegratorRange(-Izone, Izone);
      tiltLockController.setTolerance(.5);
   }

}
