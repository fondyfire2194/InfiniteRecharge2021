/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class RearIntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new RearIntake.
   */
  private final TalonSRX intakeMotor = new TalonSRX(IntakeConstants.REAR_MOTOR);
  private final DoubleSolenoid intakeArm = new DoubleSolenoid(1, 5);

public List<BaseTalon> intakeTalons;

  public RearIntakeSubsystem() {
    // intakeMotor.configFactoryDefault();

    // intakeTalons.add(intakeMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntakeMotor(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setBrakeOn(boolean on){
    if(on){
      intakeMotor.setNeutralMode(NeutralMode.Brake);
    }
    else{
      intakeMotor.setNeutralMode(NeutralMode.Coast);
    }
  }
  public void raiseIntakeArm(){
    intakeArm.set(DoubleSolenoid.Value.kReverse);
  }
  public void lowerIntakeArm(){
    intakeArm.set(DoubleSolenoid.Value.kForward);
  }
}
