// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.chassis.commands.Drive;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.commands.climb.ClimbCommand;
import frc.robot.commands.climb.OpenClimb;
import frc.robot.commands.gripper.GrabAndDrop;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Gripper;
import frc.robot.utils.CommandController;
import frc.robot.utils.CommandController.ControllerType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  public static Climb climb;
  public static Chassis chassis; 
  public static Gripper gripper;
  public static boolean isRed;
  public static Drive drive;
  public static ClimbCommand climbCommand;
  public static GrabAndDrop grabAndDrop;
  public static boolean isComp = DriverStation.isFMSAttached();
  public static CommandController driverController;
  public static CommandController operatorController;
  public static CommandXboxController controller;
 
  public RobotContainer() {
    climb = new Climb();
    driverController = new CommandController(OperatorConstants.DRIVER_CONTROLLER_PORT, ControllerType.kXbox);
    operatorController = new CommandController(OperatorConstants.OPERATOR_CONTROLLER_PORT, ControllerType.kXbox);
    controller = new CommandXboxController(0);
    chassis = new Chassis();
    drive = new Drive(chassis, driverController);
    climbCommand = new ClimbCommand(climb, driverController);
    chassis.setDefaultCommand(drive);
    climb.setDefaultCommand(climbCommand);
    gripper = new Gripper();

    controller.a().onTrue(new OpenClimb(climb));
    controller.b().onTrue(new GrabAndDrop(gripper));

  }

  public static boolean isRed() {
    return isRed;
  }


  public static boolean isComp() {
    return isComp;
  }


  public Command getAutonomousCommand() {
    return null;
  }

}
