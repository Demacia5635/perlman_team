// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.TalonMotor;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private TalonMotor Arm;
  private TalonMotor griper;
  private DigitalInput magneticSensor;
  private DutyCycleEncoder grippEncoder;
  private boolean isCalib;
  public Arm() {
    Arm = new TalonMotor(Constants.ARM_CONSTANTS.ARM_CONFIG);
    griper = new TalonMotor(Constants.ARM_CONSTANTS.GRIPPER_CONFIG);
    magneticSensor = new DigitalInput(Constants.ARM_CONSTANTS.MAGNET_CHANEL);
    grippEncoder = new DutyCycleEncoder(Constants.ARM_CONSTANTS.ABS_CANCODER);
    isCalib = false;
  }
  public boolean isCalib(){
    return isCalib;
  }
  public void calib(){
    this.isCalib = true;
  }

  public void CalibArmAngle(double angle){
    Arm.setPosition(angle);
  }

  public boolean getMagneticSensor(){
    return !magneticSensor.get();
  }
  public double getGriperAngle(){
    return (grippEncoder.get() * 2 * Math.PI) - Constants.ARM_CONSTANTS.ENCODER_BASE_ANGLE;
  }
  public double getGripperMotorAngle(){
    return griper.getCurrentPosition();
  }
  public void setPositionVoltage(double armAngle, double gripperAngle) {
    Arm.setPositionVoltage(armAngle);
    griper.setPositionVoltage(getGripperMotorAngle() + gripperAngle - getGriperAngle());
  } 
  public void armAngleNeutralMode(boolean isBrake) {
    Arm.setNeutralMode(isBrake);
  }

  public void gripperAngleNeutralMode(boolean isBrake) {
    griper.setNeutralMode(isBrake);
  }

  public void armAngleMotorSetPower( double power){
    Arm.set(power);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
