// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path.Trajectory;

import static frc.robot.vision.utils.VisionConstants.O_TO_TAG;
import java.util.ArrayList;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Path.Utils.PathPoint;
import frc.robot.chassis.commands.auto.AutoUtils;
import frc.robot.chassis.commands.auto.FieldTarget;
import frc.robot.chassis.commands.auto.FieldTarget.ELEMENT_POSITION;
import frc.robot.chassis.commands.auto.FieldTarget.FEEDER_SIDE;
import frc.robot.chassis.commands.auto.FieldTarget.LEVEL;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.gripper.commands.Drop;
import frc.robot.robot1.gripper.commands.Grab;
import frc.robot.utils.LogManager;
import frc.robot.utils.Utils;

public class FollowTrajectory extends Command {
  private Chassis chassis;
  private DemaciaTrajectory trajectory;
  private ArrayList<PathPoint> points;
  private Rotation2d wantedAngle;
  private FieldTarget target;
  private boolean usePoints;
  private boolean isScoring;
  private boolean useElasticTarget;
  private Command grabCommand;
  private Timer waitToDropTimer = new Timer();

  public FollowTrajectory(Chassis chassis, boolean isScoring) {
    this.chassis = chassis;
    this.usePoints = false;
    this.isScoring = isScoring;
    this.useElasticTarget = true;

    addRequirements(chassis);
    // this.points.add(target.getFinishPoint());
  }

  public FollowTrajectory(Chassis chassis, FieldTarget newTarget) {
    this.chassis = chassis;
    this.target = newTarget;
    this.useElasticTarget = false;
    this.usePoints = false;
    addRequirements(chassis);
  }

  public FollowTrajectory(Chassis chassis, ArrayList<PathPoint> points, Rotation2d wantedAngle) {
    this.chassis = chassis;
    this.points = points;
    this.wantedAngle = wantedAngle;
    this.usePoints = true;
    this.useElasticTarget = false;
    addRequirements(chassis);
  }

  private FieldTarget getClosestFeeder() {
    // ChassisSpeeds currSpeeds = chassis.getChassisSpeedsFieldRel();
    // Translation2d vecVel = new Translation2d(currSpeeds.vxMetersPerSecond, currSpeeds.vyMetersPerSecond);
    double distanceFromLeft = chassis.getPose().getTranslation().getDistance(O_TO_TAG[POSITION.FEEDER_LEFT.getId()]);
    double distanceFromRight = chassis.getPose().getTranslation().getDistance(O_TO_TAG[POSITION.FEEDER_RIGHT.getId()]);

    if (distanceFromLeft > distanceFromRight) {
      return new FieldTarget(POSITION.FEEDER_RIGHT, FieldTarget.getFeeder(RobotContainer.currentFeederSide, POSITION.FEEDER_RIGHT), LEVEL.FEEDER);
    } else {
      return new FieldTarget(POSITION.FEEDER_LEFT, FieldTarget.getFeeder(RobotContainer.currentFeederSide, POSITION.FEEDER_LEFT), LEVEL.FEEDER);
    }
  }

  @Override
  public void initialize() {
    RobotContainer.robot1Strip.setAutoPath();
    if (useElasticTarget) {
      this.target = isScoring ? RobotContainer.scoringTarget : getClosestFeeder();
    }

    if (!usePoints) {
      points = new ArrayList<PathPoint>();
      this.wantedAngle = target.getFinishPoint().getRotation();
      points.add(PathPoint.kZero);

      if(!target.isInRange() || (target.level.equals(FieldTarget.LEVEL.FEEDER) && !DriverStation.isAutonomous())) points.add(target.getApproachingPoint());
      // LogManager.log("APPROACH: " + points.get(points.size() - 1));

      points.add(target.getFinishPoint());
      if (target.level == LEVEL.FEEDER) {
        grabCommand = new Grab(RobotContainer.gripper)
            .andThen(new InstantCommand(() -> RobotContainer.arm.setState(ARM_ANGLE_STATES.STARTING)));
        grabCommand.schedule();
      }

      this.trajectory = new DemaciaTrajectory(points, false, wantedAngle, chassis.getPose());

    } else
      this.trajectory = new DemaciaTrajectory(points, false, wantedAngle, chassis.getPose());
    if (this.target != null)
      RobotContainer.arm.setState(Arm.levelStateToArmState(target.level));
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = chassis.getChassisSpeedsFieldRel();
    chassis.setVelocitiesWithAccel(trajectory.calculate(chassis.getPose(), Utils.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)));
    if (this.target != null) RobotContainer.arm.setState(Arm.levelStateToArmState(target.level));
    // double disFromReef = chassis.getPose().getTranslation().getDistance(RobotContainer.isRed() ? AutoUtils.redReefCenter : AutoUtils.blueReefCenter);
    // if (target != null && (disFromReef >= 1.4 && disFromReef <= 4 || target.level == LEVEL.FEEDER)) {
    //   RobotContainer.arm.setState(Arm.levelStateToArmState(target.level));
    // }
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted && !usePoints) {

      if (target.level == LEVEL.FEEDER) {
        RobotContainer.currentFeederSide = FEEDER_SIDE.MIDDLE;
        chassis.stop();
      }

      if (target.elementPosition == ELEMENT_POSITION.CORAL_LEFT
          || target.elementPosition == ELEMENT_POSITION.CORAL_RIGHT) {

        chassis.stop();
        if (DriverStation.isAutonomous()) {
        } else {
          waitToDropTimer.start();
          new WaitUntilCommand(() -> RobotContainer.arm.isReady() && waitToDropTimer.hasElapsed(0.3))
              .andThen(new Drop(RobotContainer.gripper),
                  new RunCommand(() -> chassis.setRobotRelSpeedsWithAccel(new ChassisSpeeds(-3, 0, 0)), chassis)
                      .withTimeout(0.3), new InstantCommand(()-> {waitToDropTimer.reset(); waitToDropTimer.stop();}))
              .schedule();
        }
      }

      if (target.elementPosition == ELEMENT_POSITION.ALGEA) {
        if(!DriverStation.isAutonomous()) 
          AutoUtils.removeAlgae(target.level == LEVEL.ALGAE_TOP, false).andThen(new WaitCommand(0.2), 
          new FollowTrajectory(chassis, new FieldTarget(target.position, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L3))).schedule();
        else chassis.stop();
      }

    } else if (!DriverStation.isAutonomous())
      chassis.stop();
  }

  @Override
  public boolean isFinished() {
    return (target == null || RobotContainer.arm.getState().equals(Arm.levelStateToArmState(target.level)))
      && (trajectory.isFinishedTrajectory()) ||
        (!usePoints
            && (target.level == LEVEL.FEEDER)
            && RobotContainer.gripper.isCoral());
  }
}
