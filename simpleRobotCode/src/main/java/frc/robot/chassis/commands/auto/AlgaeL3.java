package frc.robot.chassis.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Path.Trajectory.FollowTrajectory;
import frc.robot.chassis.commands.auto.FieldTarget.ELEMENT_POSITION;
import frc.robot.chassis.commands.auto.FieldTarget.LEVEL;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.gripper.commands.AlignCoral;
import frc.robot.robot1.gripper.commands.Drop;
import frc.robot.robot1.gripper.subsystems.Gripper;

public class AlgaeL3 extends SequentialCommandGroup {
    
    public AlgaeL3(Chassis chassis, Arm arm, Gripper gripper) {
        FieldTarget eAlgae = new FieldTarget(POSITION.E, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_BOTTOM);
        FieldTarget eCoral = new FieldTarget(POSITION.E, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L3);

        addCommands(
            new FollowTrajectory(chassis, eAlgae),
            new WaitCommand(0.08),
            AutoUtils.removeAlgae(false, false)
            .alongWith(new InstantCommand(()-> new AlignCoral(gripper).schedule())),
            new InstantCommand(() -> chassis.stop()),
            new WaitCommand(0.1).alongWith(new InstantCommand(() -> arm.setState(ARM_ANGLE_STATES.L3))),
            new FollowTrajectory(chassis, eCoral),
            new WaitUntilCommand(() -> !gripper.isCoralUpSensor()).alongWith(new InstantCommand(() -> new Drop(gripper).schedule())),
            new RunCommand(()-> chassis.setRobotRelVelocities(new ChassisSpeeds(-1, 0, 0)), chassis).withTimeout(0.3),
            new InstantCommand(() -> chassis.stop())

        );
    }
}
