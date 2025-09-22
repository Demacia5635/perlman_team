package frc.robot.leds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotContainer;
import frc.robot.chassis.commands.Drive;
import frc.robot.chassis.commands.auto.FieldTarget;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.leds.subsystems.LedStrip;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.gripper.subsystems.Gripper;
import frc.robot.utils.LogManager;
import frc.robot.vision.utils.VisionConstants;

public class Robot1Strip extends LedStrip {

    private final Chassis chassis;
    private final Arm arm;
    private final Gripper gripper;

    private final Timer grabTimer;
    private final Timer dropTimer;
    private final Timer autoPathTimer;
    private final Timer coralStationTimer;
    private final Timer userButtonTimer;
    private final Timer notPrecisionModeTimer;

    public boolean isManual;
    
    public Robot1Strip(Chassis chassis,Arm arm, Gripper gripper) {
        super("Robot 1 Strip", LedConstants.LENGTH, RobotContainer.ledManager);
        
        this.chassis = chassis;
        this.arm = arm;
        this.gripper = gripper;

        this.grabTimer = new Timer();
        this.dropTimer = new Timer();
        this.autoPathTimer = new Timer();
        this.coralStationTimer = new Timer();
        this.userButtonTimer = new Timer();
        this.notPrecisionModeTimer = new Timer();

        this.isManual = false;
    }

    public void setCoralStation() {
        coralStationTimer.start();
        // LogManager.log("Scoring to: " + RobotContainer.scoringTarget.toString() + " Arm Angle: " + arm.getArmAngle() + " Gripper Angle: " + arm.getGripperAngle());
    }

    public void setAutoPath() {
        autoPathTimer.start();
    }

    public void setDrop() {
        dropTimer.start();
    }

    public void setGrab() {
        grabTimer.start();
    }
    
    public void setUserButton() {
        userButtonTimer.start();
    }

    public void setNotPrecisionMode() {
        notPrecisionModeTimer.start();
    }

    public void setManualOrAuto() {
        isManual = !isManual;
    }

    @Override
    public void periodic() {

        setColor(Color.kWhite);

        if (arm.getState().equals(ARM_ANGLE_STATES.CORAL_STATION)) {
            setColor(Color.kYellow);
        }
        
        if (!grabTimer.hasElapsed(1) && grabTimer.get() != 0) {
            setBlink(Color.kYellow);
        } else if (grabTimer.hasElapsed(1)) {
            grabTimer.stop();
            grabTimer.reset();
        }

        if (!dropTimer.hasElapsed(1) && dropTimer.get() != 0) {
            setBlink(Color.kPurple);
        } else if (dropTimer.hasElapsed(1)) {
            dropTimer.stop();
            dropTimer.reset();
        }

        if (gripper.isCoral()) {
            setColor(Color.kPurple);
        }

        if (isManual) {
            FieldTarget fieldTarget = RobotContainer.scoringTarget;
            Translation2d robotToTag = chassis.getPose().getTranslation().minus(fieldTarget.getFinishPoint().getTranslation()).rotateBy(VisionConstants.TAG_ANGLE[fieldTarget.position.getId()].unaryMinus());
            double robotToTargetAngle = robotToTag.getAngle().getRadians();
            
            if (robotToTag.getNorm() <= 0.05) {
                setColor(Color.kViolet);
            } else {
                if (robotToTargetAngle > 0) {
                    setColor(setColorSides(new Color(0, robotToTag.getNorm(), 0), Color.kRed));
                } else if (robotToTargetAngle < 0) {
                    setColor(setColorSides(Color.kRed, new Color(0, robotToTag.getNorm(), 0)));
                }
            }
        }

        if (Drive.getPrecisionMode()) {
            setBlink(Color.kGreen);
        }

        if (!notPrecisionModeTimer.hasElapsed(0.75) && notPrecisionModeTimer.get() != 0) {
            setBlink(Color.kWhite);
        } else if (notPrecisionModeTimer.hasElapsed(0.75)) {
            notPrecisionModeTimer.stop();
            notPrecisionModeTimer.reset();
        }
            
        if (!autoPathTimer.hasElapsed(1) && autoPathTimer.get() != 0) {
            setBlink(Color.kRed);
        } else if (autoPathTimer.hasElapsed(1)) {
            autoPathTimer.stop();
            autoPathTimer.reset();
        }

        if (!userButtonTimer.hasElapsed(0.5) && userButtonTimer.get() != 0) {
            setBlink(Color.kOrangeRed);
        } else if (userButtonTimer.hasElapsed(0.5)) {
            userButtonTimer.stop();
            userButtonTimer.reset();
        }

        if (!coralStationTimer.hasElapsed(0.75) && coralStationTimer.get() != 0) {
            setBlink(Color.kChocolate);
        } else if (coralStationTimer.hasElapsed(0.75)) {
            coralStationTimer.stop();
            coralStationTimer.reset();
        }

        if (DriverStation.getMatchTime() <= 30 && DriverStation.getMatchTime() > 28 && !DriverStation.isAutonomous()) {
            setBlink(Color.kBlue);
        }
    }

    Color[] setColorSides(Color left, Color right) {
        Color[] colorArr = new Color[LedConstants.LENGTH];
        int i;
        for (i = 0; i < 6; i++) {
            colorArr[i] = right;
        }
        for (; i < LedConstants.LENGTH; i++) {
            colorArr[i] = left;
        }
        return colorArr;
    }
}
