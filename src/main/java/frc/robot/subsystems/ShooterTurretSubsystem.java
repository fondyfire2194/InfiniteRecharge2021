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

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodedShooterConstants;

public class ShooterTurretSubsystem extends SubsystemBase {
   /**
    * Creates a new ShooterTurret.
    */

   public WPI_TalonSRX m_rotateMotor = new WPI_TalonSRX(HoodedShooterConstants.ROTATE_MOTOR);
   private final Encoder m_rotateEncoder = new Encoder(2, 3);
   private final PIDController m_turretLockController = new PIDController(.03, 0, 0);

   public String kEnable;
   public String kDisable;
   public double requiredTurretAngle;

   public double targetHorizontalOffset;
   public double commandAngle;
   private int displaySelect;
   public boolean lockOnTarget;
   public boolean changeLocked;

   private double degreesPerEncoderCount = 1;
   public double positionCommandAngle;

   private static final double MAX_PCT_OUTPUT = 1.0;

   private static final int TALON_TIMEOUT_MS = 20;
   public static final int TICKS_PER_REVOLUTION = 2048; //

   private static final double MOVE_PROPORTIONAL_GAIN = 0.4;
   private static final double MOVE_INTEGRAL_GAIN = 0.0;
   private static final double MOVE_DERIVATIVE_GAIN = 0.0;
   static final int TALON_TICK_THRESH = 128;
   static final double TICK_THRESH = TALON_TICK_THRESH * 4;
   public static final double TICK_PER_100MS_THRESH = 64; // about a tenth of a rotation per second
   static final int PRIMARY_PID_LOOP = 0;
   private final static int MOVE_ON_TARGET_MINIMUM_COUNT = 20; // number of times/iterations we need to be on target to
                                                               // really be on target

   private final static int MOVE_STALLED_MINIMUM_COUNT = MOVE_ON_TARGET_MINIMUM_COUNT * 2 + 30; // number of
                                                                                                // times/iterations we
                                                                                                // need to be stalled to
                                                                                                // really be stalled

   public ShooterTurretSubsystem() {
      m_rotateMotor.set(ControlMode.PercentOutput, 0);
      m_rotateMotor.configFactoryDefault();
      m_rotateMotor.setNeutralMode(NeutralMode.Brake);
      m_rotateMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PRIMARY_PID_LOOP, TALON_TIMEOUT_MS);

      setPIDParameters();
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

   public double getTurretEncoderRevs() {
      return m_rotateEncoder.getDistance();
   }

   public double getTurretOut() {
      return m_rotateMotor.get();
   }

   public void resetTurretPosition() {
      m_rotateEncoder.reset();
      commandAngle = 0;
   }

   public double getTurretAngle() {
      return m_rotateEncoder.getDistance();
   }

   public double getTurretSpeed() {
      return m_rotateEncoder.getRate();
   }

   public void jogTurret(double speed) {
      commandAngle = m_rotateEncoder.getDistance();
      m_rotateMotor.set(ControlMode.PercentOutput, speed);
   }

   public void positionTurretToAngle(double angle) {
      commandAngle = angle;
      m_rotateMotor.set(ControlMode.Position, angle);
   }

   public boolean lockTurretToVision(double cameraError) {
      double pidOut = m_turretLockController.calculate(cameraError, 0);
      m_rotateMotor.set(ControlMode.PercentOutput, pidOut);
      SmartDashboard.putNumber("PIDO", pidOut);
      commandAngle = getTurretAngle();
      SmartDashboard.putString("TurretState", "VisionLock");
      return m_turretLockController.atSetpoint();
   }

public boolean turretInPosition(){

return Math.abs(commandAngle-getTurretAngle())<2;

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

   public void setPIDParameters() {
      m_rotateMotor.configAllowableClosedloopError(0, TALON_TICK_THRESH, TALON_TIMEOUT_MS);

      // P is the proportional gain. It modifies the closed-loop output by a
      // proportion (the gain value)
      // of the closed-loop error.
      // P gain is specified in output unit per error unit.
      // When tuning P, it's useful to estimate your starting value.
      // If you want your mechanism to drive 50% output when the error is 4096 (one
      // rotation when using CTRE Mag Encoder),
      // then the calculated Proportional Gain would be (0.50 X 1023) / 4096 = ~0.125.

      // I is the integral gain. It modifies the closed-loop output according to the
      // integral error
      // (summation of the closed-loop error each iteration).
      // I gain is specified in output units per integrated error.
      // If your mechanism never quite reaches your target and using integral gain is
      // viable,
      // start with 1/100th of the Proportional Gain.

      // D is the derivative gain. It modifies the closed-loop output according to the
      // derivative error
      // (change in closed-loop error each iteration).
      // D gain is specified in output units per derivative error.
      // If your mechanism accelerates too abruptly, Derivative Gain can be used to
      // smooth the motion.
      // Typically start with 10x to 100x of your current Proportional Gain.

      m_rotateMotor.config_kP(0, MOVE_PROPORTIONAL_GAIN, TALON_TIMEOUT_MS);
      m_rotateMotor.config_kI(0, MOVE_INTEGRAL_GAIN, TALON_TIMEOUT_MS);
      m_rotateMotor.config_kD(0, MOVE_DERIVATIVE_GAIN, TALON_TIMEOUT_MS);
      m_rotateMotor.config_kF(0, 0, TALON_TIMEOUT_MS);

   }

}
