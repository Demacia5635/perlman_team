// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.chassis.commands.Drive;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.CommandController;
import frc.robot.utils.CommandController.ControllerType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {

  public static Chassis chassis; 
  public static boolean isRed;
  public static boolean isComp = DriverStation.isFMSAttached();
  public static CommandController driverController;
  public static CommandController operatorController;
  public static CommandXboxController controller;
 
  public RobotContainer() {
    driverController = new CommandController(OperatorConstants.DRIVER_CONTROLLER_PORT, ControllerType.kXbox);
    operatorController = new CommandController(OperatorConstants.OPERATOR_CONTROLLER_PORT, ControllerType.kXbox);
    controller = new CommandXboxController(0);
    chassis = new Chassis();
    Drive drive = new Drive(chassis, driverController);
    chassis.setDefaultCommand(drive);
    //controller.a().onTrue(new MoveArm(null));

  }

  public static boolean isRed() {
    return isRed;
  }


  public static boolean isComp() {
    return isComp;
  }


  public Command getAutonomousCommand() {
    chassis.setDefaultCommand(new Drive(chassis, driverController));

    return null;
  }

}
