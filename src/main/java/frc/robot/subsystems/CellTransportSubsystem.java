/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CellTransportConstants;

public class CellTransportSubsystem extends SubsystemBase {
  /**
   * Creates a new CellTransport.
   */
  private final TalonSRX leftBeltMotor = new TalonSRX(CellTransportConstants.LEFT_BELT_MOTOR);
  private final TalonSRX rightBeltMotor = new TalonSRX(CellTransportConstants.RIGHT_BELT_MOTOR);
  private final TalonSRX frontRollerMotor = new TalonSRX(CellTransportConstants.FRONT_ROLLER);
  private final TalonSRX rearRollerMotor = new TalonSRX(CellTransportConstants.REAR_ROLLER);

  public List<BaseTalon> transportTalons;

  
  private double beltPulseStartTime;

  public CellTransportSubsystem() {
    leftBeltMotor.configFactoryDefault();
    rightBeltMotor.configFactoryDefault(); 
    frontRollerMotor.configFactoryDefault();
    rearRollerMotor.configFactoryDefault();

    setFrontRollerBrakeOn(true);
    setRearRollerBrakeOn(true);
    setBeltBrakeOn(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    

  }

  public void runLeftBeltMotor(double speed) {
    leftBeltMotor.set(ControlMode.PercentOutput, speed);
  }

  public void runRightBeltMotor(double speed) {
    leftBeltMotor.set(ControlMode.PercentOutput, speed);
  }

  public void pulseBelt(double speed, double onTime, double offTime) {
    if (beltPulseStartTime == 0)
      beltPulseStartTime = Timer.getFPGATimestamp();

    if (Timer.getFPGATimestamp() < beltPulseStartTime + onTime) {
      runLeftBeltMotor(-speed);
    }

    if (Timer.getFPGATimestamp() > beltPulseStartTime + onTime
        && Timer.getFPGATimestamp() < beltPulseStartTime + onTime + offTime) {
      runLeftBeltMotor(0);
    }

    if (Timer.getFPGATimestamp() > beltPulseStartTime + onTime + offTime) {
      beltPulseStartTime = 0;
    }

  }

  public void setBeltBrakeOn(boolean on) {
    if (on) {
      leftBeltMotor.setNeutralMode(NeutralMode.Brake);
      rightBeltMotor.setNeutralMode(NeutralMode.Brake);     
    } else {
      leftBeltMotor.setNeutralMode(NeutralMode.Coast);
      rightBeltMotor.setNeutralMode(NeutralMode.Coast);  
    }
  }

  public void runFrontRollerMotor(double speed) {
    frontRollerMotor.set(ControlMode.PercentOutput, -speed);
  }

  public void setFrontRollerBrakeOn(boolean on) {
    if (on) {
      frontRollerMotor.setNeutralMode(NeutralMode.Brake);
    } else {
      frontRollerMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void runRearRollerMotor(double speed) {
    rearRollerMotor.set(ControlMode.PercentOutput, -speed);
  }

  public void setRearRollerBrakeOn(boolean on) {
    if (on) {
      rearRollerMotor.setNeutralMode(NeutralMode.Brake);
    } else {
      rearRollerMotor.setNeutralMode(NeutralMode.Coast);
    }
  }
}
