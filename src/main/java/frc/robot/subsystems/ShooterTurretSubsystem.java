/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodedShooterConstants;

public class ShooterTurretSubsystem extends SubsystemBase {
   /**
    * Creates a new ShooterTurret.
    */

   public CANSparkMax m_rotateMotor = new CANSparkMax(HoodedShooterConstants.ROTATE_MOTOR, MotorType.kBrushless);
   private final CANPIDController m_rotateController = new CANPIDController(m_rotateMotor);
   private final CANEncoder m_rotateEncoder = m_rotateMotor.getEncoder();
   private final PIDController m_turretLockController = new PIDController(.03, 0, 0);

   public String kEnable;
   public String kDisable;
   public double requiredTurretAngle;

   public double targetHorizontalOffset;
   public double commandTurns;
   private int displaySelect;
   public boolean lockOnTarget;
   public boolean changeLocked;

   private CANDigitalInput m_forwardLimit;
   private CANDigitalInput m_reverseLimit;
   public double positionCommandAngle;

   public List<Integer> sparkMaxID;
   public List<CANSparkMax> motor;

   public ShooterTurretSubsystem() {
      m_rotateMotor.restoreFactoryDefaults(true);
      m_rotateMotor.setClosedLoopRampRate(50);
      m_rotateMotor.setOpenLoopRampRate(2);

      m_rotateMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
            (float) HoodedShooterConstants.TURRET_MAX_TURNS);
      m_rotateMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
            (float) HoodedShooterConstants.TURRET_MIN_TURNS);
      m_rotateMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      m_rotateMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
      m_rotateMotor.setIdleMode(IdleMode.kBrake);

      // motor.add(m_rotateMotor);
      // sparkMaxID.add(m_rotateMotor.getDeviceId());

      // m_forwardLimit =
      // m_rotateMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
      // m_reverseLimit =
      // m_rotateMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
      // m_forwardLimit.enableLimitSwitch(false);
      // m_reverseLimit.enableLimitSwitch(false);
      setTurretGains();
      resetTurretPosition();
      commandTurns = 0;
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
         SmartDashboard.putNumber("Turret Revs", getTurretEncoderRevs());
         SmartDashboard.putNumber("Turret OUT", getTurnOut());
         SmartDashboard.putNumber("Turret Command", commandTurns);
         SmartDashboard.putNumber("Turret Amps", m_rotateMotor.getOutputCurrent());

      }
   }

   public double getTurretEncoderRevs() {
      return m_rotateEncoder.getPosition();
   }

   public double getTurnOut() {
      return m_rotateMotor.get();
   }

   public void resetTurretPosition() {
      m_rotateEncoder.setPosition(0.);
      commandTurns = 0;
   }

   public double getTurretPosition() {
      return m_rotateEncoder.getPosition();
   }

   public double getTurretSpeed() {
      return m_rotateEncoder.getVelocity();
   }

   public double getTurretAngle() {
      return getTurretEncoderRevs() * HoodedShooterConstants.TURRET_ENCODER_DEG_PER_REV;
   }

   public void jogTurret(double speed) {
      commandTurns = m_rotateEncoder.getPosition();
      m_rotateMotor.set(speed);
   }

   public void jogTurretVel(double speed) {
      commandTurns = m_rotateEncoder.getPosition();
      m_rotateController.setReference(speed, ControlType.kVelocity);
      SmartDashboard.putString("TurretState", "Jogging");
   }

   public void positionTurretToAngle(double angle) {
      commandTurns = angle / HoodedShooterConstants.TURRET_ENCODER_DEG_PER_REV;
      positionTurretToTurns();
   }

   public boolean lockTurretToVision(double cameraError) {
      double pidOut = m_turretLockController.calculate(cameraError, 0);
      m_rotateMotor.set(pidOut);
      SmartDashboard.putNumber("PIDO", pidOut);
      commandTurns = getTurretPosition();
      SmartDashboard.putString("TurretState", "VisionLock");
      return m_turretLockController.atSetpoint();
   }

   public void positionTurretToTurns() {
      m_rotateController.setReference(commandTurns, ControlType.kSmartMotion);
      SmartDashboard.putString("TurretState", "Positioning");
   }

   public void positionTurret(double turns) {
      commandTurns = turns;
      positionTurretToTurns();
   }

   public boolean getRotateInPosition() {
      return false;
   }

   public void changeTurretOffset(boolean right) {
      if (right)
         targetHorizontalOffset += .25;
      else
         targetHorizontalOffset -= .25;
   }

   public double iterateTurretPosition(double speed) {

      return commandTurns + speed / 100;
   }

   public void setTurretGains() {
      double kP = 6e-5;
      double kI = 1e-6;
      double kD = 0;
      double kIz = 10;
      double kFF = 0.0002;
      double kMaxOutput = .25;
      double kMinOutput = -.25;
      double maxRPM = 5700;

      // Smart Motion Coeffic ients
      double maxVel = 1500; // rpm
      double maxAcc = 2500;
      double minVel = 0;
      double allowedErr = 0;

      // set PID coefficients
      m_rotateController.setP(kP);
      m_rotateController.setI(kI);
      m_rotateController.setD(kD);
      m_rotateController.setIZone(kIz);
      m_rotateController.setFF(kFF);
      m_rotateController.setOutputRange(kMinOutput, kMaxOutput);
      int smartMotionSlot = 0;
      m_rotateController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
      m_rotateController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
      m_rotateController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
      m_rotateController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
   }

}
