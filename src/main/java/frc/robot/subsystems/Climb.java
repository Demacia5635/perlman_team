package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.utils.TalonMotor;

public class Climb extends SubsystemBase {
  private TalonMotor climbMotor;
  private DigitalInput angleLimit;
  public Climb() {

    climbMotor = new TalonMotor(ClimbConstants.MOTOR_CONFIG);
    angleLimit = new DigitalInput(ClimbConstants.LIMIT_SWITCH_CHANNEL);

  }

  public void setClimbPower(double power){
    climbMotor.set(power);
  }

  public void stopClimb(){
    climbMotor.stopMotor();
  }

  public void breakClimb(){
    climbMotor.setNeutralMode(true);
  }
  
  public void coastClimb(){
    climbMotor.setNeutralMode(false);
  }

  public double getArmAngle(){
    return climbMotor.getCurrentPosition();
  }

  public boolean getLimit() {
    return !angleLimit.get();
  }

  public void setAngle(double angle){
    climbMotor.setPosition(angle);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
  
  @Override
  public void periodic() {
    
  }
}