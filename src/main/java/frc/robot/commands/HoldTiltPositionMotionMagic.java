package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.ShooterTiltSubsystem;
import frc.robot.Pref;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *
 */
public class HoldTiltPositionMotionMagic extends CommandBase {
	private double lastHoldPositionDegrees;
	private final ShooterTiltSubsystem m_tilt;

	public HoldTiltPositionMotionMagic(ShooterTiltSubsystem tilt) {

		m_tilt = tilt;
		addRequirements(m_tilt);

	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		m_tilt.m_tiltMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
		m_tilt.m_tiltMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
		m_tilt.m_tiltMotor.selectProfileSlot(0, 0);

		m_tilt.m_tiltMotor.configOpenloopRamp(0, 0);
		m_tilt.m_tiltMotor.configClosedloopRamp(0, 0);
		m_tilt.m_tiltMotor.configPeakOutputForward(1, 0);
		m_tilt.m_tiltMotor.configPeakOutputReverse(-1, 0);
		m_tilt.m_tiltMotor.configNominalOutputForward(0, 0);
		m_tilt.m_tiltMotor.configNominalOutputReverse(0, 0);
		m_tilt.m_tiltMotor.selectProfileSlot(0, 0);

		m_tilt.m_tiltMotor.config_kF(0, Pref.getPref("ElevatorMMKf"), 0);
		m_tilt.m_tiltMotor.config_kP(0, Pref.getPref("ElevatorMMKp"), 0);
		m_tilt.m_tiltMotor.config_kI(0, Pref.getPref("ElevatorMMKi"), 0);
		m_tilt.m_tiltMotor.config_kD(0, Pref.getPref("ElevatorMMKd"), 0);

		m_tilt.holdPositionDegrees = m_tilt.getTiltAngle();
		m_tilt.lastHoldPositionDegrees = m_tilt.holdPositionDegrees + .01;

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {

		if (m_tilt.holdPositionDegrees != lastHoldPositionDegrees) {
			m_tilt.motionMagic(m_tilt.holdPositionDegrees);
			lastHoldPositionDegrees = m_tilt.holdPositionDegrees;
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

