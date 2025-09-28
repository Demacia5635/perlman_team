package frc.robot.chassis.subsystems;

import static frc.robot.vision.utils.VisionConstants.*;

import java.util.List;

import org.ejml.simple.SimpleMatrix;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import static frc.robot.chassis.utils.ChassisConstants.*;

import frc.robot.chassis.utils.SwerveKinematics;
import frc.robot.utils.LogManager;
import frc.robot.utils.Utils;
import frc.robot.vision.Camera;
import frc.robot.vision.Camera.CameraType;
import frc.robot.vision.subsystem.Tag;
import frc.robot.vision.utils.VisionFuse;

public class Chassis extends SubsystemBase {
    private SwerveModule[] modules;
    private Pigeon2 gyro;
    private SwerveKinematics kinematicsFix;
    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d field;

    public Tag reefRight;
    public Tag feeder;
    public Tag barge;
    public Tag reefLeft;

    public VisionFuse visionFuse;

    private StatusSignal<Angle> gyroYawStatus;
    private Rotation2d lastGyroYaw;

    public Chassis() {
        modules = new SwerveModule[] {
                new SwerveModule(FRONT_LEFT),
                new SwerveModule(FRONT_RIGHT),
                new SwerveModule(BACK_LEFT),
                new SwerveModule(BACK_RIGHT),
        };
        gyro = new Pigeon2(GYRO_ID, GYRO_CAN_BUS);
        addStatus();
        kinematicsFix = new SwerveKinematics(
                FRONT_LEFT.POSITION,
                FRONT_RIGHT.POSITION,
                BACK_LEFT.POSITION,
                BACK_RIGHT.POSITION

        );
        poseEstimator = new SwerveDrivePoseEstimator(kinematicsFix, getGyroAngle(), getModulePositions(), new Pose2d());

        SimpleMatrix std = new SimpleMatrix(new double[] { 0.02, 0.02, 0 });
        poseEstimator.setVisionMeasurementStdDevs(new Matrix<>(std));
        field = new Field2d();

        
        
        reefRight = new Tag(()->getGyroAngle(), ()->getChassisSpeedsRobotRel(), 
            new Camera("right", new Translation3d(0.14310487, -0.28932432, 0.777), 90-33, 0, CameraType.REEF));

        reefLeft = new Tag(()->getGyroAngle(), ()->getChassisSpeedsRobotRel(), 
            new Camera("left", new Translation3d(0.1475, 0.291, 0.704),  65, 0, CameraType.REEF));
       
        feeder = new Tag(()->getGyroAngle(), ()->getChassisSpeedsRobotRel(),
            new Camera("feeder", new Translation3d(0.11, -0.285, 0.91), 20, 0, CameraType.FEEDER));

        barge = new Tag(()->getGyroAngle(), ()->getChassisSpeedsRobotRel(), 
            new Camera("barge", new Translation3d(0.13, 0.284, 0.89), 53, 180, CameraType.BARGE));
            
        visionFuse = new VisionFuse(reefRight, feeder, barge, reefLeft);


        SmartDashboard.putData("reset gyro", new InstantCommand(() -> setYaw(Rotation2d.kZero)).ignoringDisable(true));
        SmartDashboard.putData("reset gyro 180", new InstantCommand(() -> setYaw(Rotation2d.kPi)).ignoringDisable(true));
        SmartDashboard.putData("set gyro to 3D tag", new InstantCommand(() -> setYaw(
                Rotation2d.fromDegrees(visionFuse.get3DAngle()))).ignoringDisable(true));
        SmartDashboard.putData("set gyro to left camera", new InstantCommand(() -> setYaw(
            Rotation2d.fromDegrees(reefLeft.getAngle())
        )).ignoringDisable(true));
        SmartDashboard.putData("set gyro to right camera", new InstantCommand(() -> setYaw(
            Rotation2d.fromDegrees(reefRight.getAngle())
        )).ignoringDisable(true));
        SmartDashboard.putData("change camera dimension", new Command() {
            private static boolean is3d = false;
            
            public void initialize() {
                visionFuse.set3D(!is3d);
                is3d = !is3d;
            };
            
            public boolean isFinished() {return true;}
            public boolean runsWhenDisabled() {return true;};
        });
        LogManager.addEntry("gyro", () -> getGyroAngle().getRadians());
        SmartDashboard.putData("field", field);

        LogManager.addEntry("VELOCITY NORM: ", () -> Utils.hypot(getChassisSpeedsRobotRel().vxMetersPerSecond, getChassisSpeedsRobotRel().vyMetersPerSecond));

        SmartDashboard.putData("Chassis/set coast", new InstantCommand(() -> setNeutralMode(false)).ignoringDisable(true));
        SmartDashboard.putData("Chassis/set brake", new InstantCommand(() -> setNeutralMode(true)).ignoringDisable(true));

    }

    public void checkElectronics() {
        for (SwerveModule module : modules) {
            module.checkElectronics();
        }
    }


    public void setNeutralMode(boolean isBrake) {
        for (SwerveModule module : modules) {
            module.setNeutralMode(isBrake);
        }
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    private void addStatus() {
        gyroYawStatus = gyro.getYaw();
        lastGyroYaw = new Rotation2d(gyroYawStatus.getValueAsDouble());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
    

    public void setVelocities(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroAngle());
        speeds = ChassisSpeeds.discretize(speeds, CYCLE_DT);
        
        SwerveModuleState[] states = kinematicsFix.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }



    public void setSteerPositions(double[] positions) {
        for (int i = 0; i < positions.length; i++) {
            modules[i].setSteerPosition(positions[i]);
        }
    }

    public void setSteerPower(double pow, int id) {
        modules[id].setSteerPower(pow);
    }

    public double getSteerVelocity(int id) {
        return modules[id].getSteerVel();
    }

    public double getSteeracceleration(int id) {
        return modules[id].getSteerAccel();
    }

    public void setSteerPositions(double position) {
        setSteerPositions(new double[] { position, position, position, position });
    }

    public ChassisSpeeds getRobotRelVelocities() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(getChassisSpeedsRobotRel(), getGyroAngle());
    }

    public void setRobotRelVelocities(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematicsFix.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }

    public void setDriveVelocities(double[] velocities) {
        for (int i = 0; i < velocities.length; i++) {
            modules[i].setDriveVelocity(velocities[i]);
        }
    }

    public void setDriveVelocities(double velocity) {
        setDriveVelocities(new double[] { velocity, velocity, velocity, velocity });
    }

    public Rotation2d getGyroAngle() {
        gyroYawStatus.refresh();
        if (gyroYawStatus.getStatus() == StatusCode.OK) {
            lastGyroYaw = new Rotation2d(gyroYawStatus.getValue());
        }
        return lastGyroYaw;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] arr = new SwerveModulePosition[modules.length];
        for (int i = 0; i < arr.length; i++) {
            arr[i] = modules[i].getModulePosition();
        }
        return arr;
    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            modules[i].setState(states[i]);
        }
    }

    private void updateVision(Pose2d pose) {
        poseEstimator.setVisionMeasurementStdDevs(getSTD());
        poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - 0.05);
    }

    private Matrix<N3, N1> getSTD() {
        double x = 0.05;
        double y = 0.05;
        double theta = 0.03;

        ChassisSpeeds currentSpeeds = getChassisSpeedsRobotRel();
        double speed = Utils.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        // Vision confidence adjustment
        if (visionFuse.getVisionConfidence() < 0.3) {
            x += 0.3;
            y += 0.3;
        }

        // Speed-based confidence calculation
        if (speed > WORST_RELIABLE_SPEED) {
            // Maximum uncertainty for high speeds
            x += 0.02;
            y += 0.02;
        } else if (speed <= BEST_RELIABLE_SPEED) {
            // Minimum uncertainty for low speeds
            x -= 0.02;
            y -= 0.02;
        } else {
            // Calculate normalized speed for the falloff range
            double normalizedSpeed = (speed - BEST_RELIABLE_SPEED)
                    / (WORST_RELIABLE_SPEED - BEST_RELIABLE_SPEED);

            // Apply exponential falloff to calculate additional uncertainty
            double speedConfidence = Math.exp(-3 * normalizedSpeed);

            // Scale the uncertainty adjustment based on confidence
            double adjustment = 0.02 * (1 - speedConfidence);
            x += adjustment;
            y += adjustment;
        }

        return new Matrix<N3, N1>(new SimpleMatrix(new double[] { x, y, theta }));
    }


    Pose2d visionFusePoseEstimation;
    Rotation2d gyroAngle;

    @Override
    public void periodic() {
        visionFusePoseEstimation = visionFuse.getPoseEstemation();
        gyroAngle = getGyroAngle();
        if (visionFusePoseEstimation != null) {
            updateVision(new Pose2d(visionFusePoseEstimation.getTranslation(), gyroAngle));
            // fieldTag.setRobotPose(visionFusePoseEstimation);
            // if (visionFuse.get2dAngle() != null){
            //     fieldTest.setRobotPose(new Pose2d(visionFusePoseEstimation.getTranslation(), visionFuse.get2dAngle()));
            // }
        }
        poseEstimator.update(gyroAngle, getModulePositions());

        field.setRobotPose(poseEstimator.getEstimatedPosition());

    }

    public boolean isRed() {
        return RobotContainer.isRed();
    }

    public ChassisSpeeds getChassisSpeedsRobotRel() {
        return kinematicsFix.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getChassisSpeedsFieldRel() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(kinematicsFix.toChassisSpeeds(getModuleStates()), getGyroAngle());
    }

    /**
     * Returns the state of every module
     * 
     * 
     * @return Velocity in m/s, angle in Rotation2d
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] res = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            res[i] = modules[i].getState();
        }
        return res;
    }

    public void setYaw(Rotation2d angle) {
        if (angle != null) {
            gyro.setYaw(angle.getDegrees());
            poseEstimator
                    .resetPose(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), gyro.getRotation2d()));
        }
    }


    PIDController drivePID = new PIDController(2, 0, 0);

    public void stop() {
        for (SwerveModule i : modules) {
            i.stop();
        }
    }
    public boolean isSeeTag(int cameraID){
        Tag[] tags = { reefRight, feeder, barge, reefLeft };

        return tags[cameraID].isSeeTag();
    }
    public boolean isSeeTag(int id, int cameraId, double distance) {
        Tag[] tags = { reefRight, feeder, barge, reefLeft };

        return tags[cameraId].isSeeTag(id, distance);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }

    public Trajectory vector(Translation2d start, Translation2d end){
      return TrajectoryGenerator.generateTrajectory(
            List.of(
              new Pose2d(start, end.getAngle().minus(start.getAngle())),
              new Pose2d(end, end.getAngle().minus(start.getAngle()))),
            new TrajectoryConfig(4.0, 4.0));
    }
}
