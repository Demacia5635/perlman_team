// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.chassis.commands.Drive;
import frc.robot.chassis.commands.auto.FieldTarget;
import frc.robot.chassis.commands.auto.FieldTarget.ELEMENT_POSITION;
import frc.robot.chassis.commands.auto.FieldTarget.FEEDER_SIDE;
import frc.robot.chassis.commands.auto.FieldTarget.LEVEL;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.leds.Robot1Strip;
import frc.robot.leds.subsystems.LedManager;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.gripper.subsystems.Gripper;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.utils.CommandController;
import frc.robot.utils.CommandController.ControllerType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  public static FEEDER_SIDE currentFeederSide;
  public static Gripper gripper;
  public static Arm arm;
  public static Chassis chassis; 
  public static LedManager ledManager;
  public static boolean isRed;
  public static Robot1Strip robot1Strip;
  public static boolean isComp = DriverStation.isFMSAttached();
  public static CommandController driverController;
  public static CommandController operatorController;

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static FieldTarget scoringTarget = new FieldTarget(POSITION.A, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L3);

 

  public RobotContainer() {
    driverController = new CommandController(OperatorConstants.DRIVER_CONTROLLER_PORT, ControllerType.kXbox);
    operatorController = new CommandController(OperatorConstants.OPERATOR_CONTROLLER_PORT, ControllerType.kXbox);

    configureBindings();
  }

  public static boolean isRed() {
    return isRed;
  }

  private void configureBindings() {


  }
  public static boolean isComp() {
    return isComp;
  }


  public Command getAutonomousCommand() {
    chassis.setDefaultCommand(new Drive(chassis, driverController));

    return Autos.exampleAuto(m_exampleSubsystem);
  }

}
