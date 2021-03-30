package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Pref;
import frc.robot.subsystems.ShooterTurretSubsystem;

/**
 *
 */
public class HoldTurretPositionMotionMagic extends CommandBase {
	private double lastHoldPositionDegrees;
	private final ShooterTurretSubsystem m_turret;

	public HoldTurretPositionMotionMagic(ShooterTurretSubsystem turret) {

		m_turret = turret;
		addRequirements(m_turret);

	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		m_turret.m_rotateMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
		m_turret.m_rotateMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
		m_turret.m_rotateMotor.selectProfileSlot(0, 0);

		m_turret.m_rotateMotor.configOpenloopRamp(0, 0);
		m_turret.m_rotateMotor.configClosedloopRamp(0, 0);
		m_turret.m_rotateMotor.configPeakOutputForward(1, 0);
		m_turret.m_rotateMotor.configPeakOutputReverse(-1, 0);
		m_turret.m_rotateMotor.configNominalOutputForward(0, 0);
		m_turret.m_rotateMotor.configNominalOutputReverse(0, 0);
		m_turret.m_rotateMotor.selectProfileSlot(0, 0);

		m_turret.m_rotateMotor.config_kF(0, Pref.getPref("TurretMMKf"), 0);
		m_turret.m_rotateMotor.config_kP(0, Pref.getPref("TurretMMKp"), 0);
		m_turret.m_rotateMotor.config_kI(0, Pref.getPref("TurretMMKi"), 0);
		m_turret.m_rotateMotor.config_kD(0, Pref.getPref("TurretMMKd"), 0);

		m_turret.holdPositionDegrees = m_turret.getTurretAngle();
		lastHoldPositionDegrees = m_turret.holdPositionDegrees + .01;

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {

		if (m_turret.holdPositionDegrees != lastHoldPositionDegrees) {
			m_turret.motionMagic(m_turret.holdPositionDegrees);
			lastHoldPositionDegrees = m_turret.holdPositionDegrees;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

