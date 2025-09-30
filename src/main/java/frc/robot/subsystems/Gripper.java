// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gripper extends SubsystemBase {
  /** Creates a new gripper. */
  private final TalonSRX motor;

  private final AnalogInput downSensor;

  public Gripper() {
    motor = new TalonSRX(Constants.GRIPER_CONSTANTS.MOTOR_ID);
    downSensor = new AnalogInput(Constants.GRIPER_CONSTANTS.DOWN_SENSOR_ID);

  }

  public void setPower(double power){
    motor.set(ControlMode.PercentOutput, power);
  }

  public boolean isIn(){
    return downSensor.getAverageValue() < Constants.GRIPER_CONSTANTS.CORAL_IN_DOWN_SENSOR;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
