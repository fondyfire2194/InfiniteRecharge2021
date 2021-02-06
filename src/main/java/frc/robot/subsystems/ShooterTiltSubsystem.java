/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Pref;
import frc.robot.Constants.HoodedShooterConstants;

public class ShooterTiltSubsystem extends SubsystemBase {
   /**
    * Creates a new ShooterTurret.
    */

   private final CANSparkMax m_tiltMotor = new CANSparkMax(HoodedShooterConstants.TILT_MOTOR, MotorType.kBrushless);

   private final CANEncoder m_tiltEncoder = m_tiltMotor.getEncoder();
   private final PIDController tiltPositionController = new PIDController(.05, 0.01, 0);
   private final PIDController tiltLockController = new PIDController(.032, 0.001, 0);
   public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

   public CANDigitalInput m_reverseLimit;

   public String kEnable;
   public String kDisable;

   public double requiredTiltAngle;

   // public double commandTurns;
   public double targetVerticalOffset;
   private int displaySelect;
   public boolean loookForTarget;
   public boolean changeLocked;

   public double positionCommandTurns;

   public boolean positionResetDone;

   private boolean switchPositionLast;

   public List<Integer> sparkMaxID;
   public List<CANSparkMax> motor;

   public ShooterTiltSubsystem() {
      m_tiltMotor.restoreFactoryDefaults();

      // motor.add(m_tiltMotor);
      // sparkMaxID.add(m_tiltMotor.getDeviceId());

      m_tiltMotor.setClosedLoopRampRate(5.);
      m_tiltMotor.setOpenLoopRampRate(1);
      m_tiltMotor.setSmartCurrentLimit(60, 60);

      m_reverseLimit = m_tiltMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
      m_reverseLimit.enableLimitSwitch(true);

      positionResetDone = false;
      targetVerticalOffset = 0;
      
      setTiltPosGains();
      setTiltLockGains();

      m_tiltMotor.setIdleMode(IdleMode.kBrake);
      if (m_reverseLimit.get()) {
         resetTiltPosition();
         positionCommandTurns = 0;
         positionResetDone = true;

         m_tiltMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
               (float) HoodedShooterConstants.TILT_MAX_TURNS);
         m_tiltMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

         m_tiltMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
               (float) HoodedShooterConstants.TILT_MIN_TURNS);
         m_tiltMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
         SmartDashboard.putNumber("Tilt Fwd Soft Limit",
               m_tiltMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));
         SmartDashboard.putNumber("Tilt Rev Soft Limit",
               m_tiltMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse));
         SmartDashboard.putBoolean("Forward Soft Limit Enabled",
               m_tiltMotor.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kForward));
         SmartDashboard.putBoolean("Reverse Soft Limit Enabled",
               m_tiltMotor.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kReverse));
      }

      SmartDashboard.putString("TiltState", "Init");

   }

   @Override
   public void periodic() {
      // This method will be called once per scheduler run

      if (m_reverseLimit.get() && (!positionResetDone || !switchPositionLast)) {
         resetTiltPosition();
         positionCommandTurns = 0;
         positionResetDone = true;

         m_tiltMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
               (float) HoodedShooterConstants.TILT_MAX_TURNS);
         m_tiltMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

         m_tiltMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
               (float) HoodedShooterConstants.TILT_MIN_TURNS);
         m_tiltMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
         SmartDashboard.putNumber("Tilt Fwd Soft Limit",
               m_tiltMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));
         SmartDashboard.putNumber("Tilt Rev Soft Limit",
               m_tiltMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse));
         SmartDashboard.putBoolean("Forward Soft Limit Enabled",
               m_tiltMotor.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kForward));
         SmartDashboard.putBoolean("Reverse Soft Limit Enabled",
               m_tiltMotor.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kReverse));

      }
      switchPositionLast = m_reverseLimit.get();

      displaySelect++;
      if (displaySelect >= 7) {
         displaySelect = 0;

         SmartDashboard.putNumber("TiltPosn", getTiltPosition());
         SmartDashboard.putNumber("Tilt Angle", getTiltAngle());
         SmartDashboard.putNumber("Tilt OUT", getTiltOut());
         SmartDashboard.putNumber("Tilt Amps", m_tiltMotor.getOutputCurrent());
         SmartDashboard.putBoolean("Tilt Down R LS", m_reverseLimit.get());
         SmartDashboard.putNumber("TiltComTrns", positionCommandTurns);
         SmartDashboard.putBoolean("TILT DOWN LIMITS", positionResetDone);

      }
   }

   public void resetTiltPosition() {
      m_tiltEncoder.setPosition(0.);
      positionCommandTurns = 0;
   }

   public void jogTilt(double speed) {
      positionCommandTurns = m_tiltEncoder.getPosition();
      SmartDashboard.putString("TiltState", "Jogging");

      m_tiltMotor.set(speed);
   }

   public void positionTilttoTurns(double turns) {
      positionCommandTurns = turns;
      positionTilttoTurns();
   }

   public void positionTilttoTurns() {

      double pidOut = tiltPositionController.calculate(getTiltPosition(), positionCommandTurns);
      m_tiltMotor.set(pidOut);
      SmartDashboard.putNumber("PIDORRTilt", pidOut);
      // positionCommandTurns = getTiltPosition();
      SmartDashboard.putNumber("TIPosErr", tiltPositionController.getPositionError());
      SmartDashboard.putString("TiltState", "Positioning");

   }

   public double getTiltOut() {
      return m_tiltMotor.get();
   }

   public double getTiltPosition() {
      return m_tiltEncoder.getPosition();
   }

   public double getTiltAngle() {
      return m_tiltEncoder.getPosition() * HoodedShooterConstants.TILT_DEG_PER_ENCODER_REV;
   };

   public boolean getTiltInPosition() {
      return true;
   }

   public double getTiltSpeed() {
      return m_tiltEncoder.getVelocity();
   }

   public boolean lockTiltToVision(double cameraError) {
      double pidOut = tiltLockController.calculate(cameraError, 0);
      m_tiltMotor.set(pidOut);
      SmartDashboard.putNumber("PIDOTilt", pidOut);
      SmartDashboard.putNumber("TILockErr", tiltLockController.getPositionError());
      positionCommandTurns = getTiltPosition();

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
