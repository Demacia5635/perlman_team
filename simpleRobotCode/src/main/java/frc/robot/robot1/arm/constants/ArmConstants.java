// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.arm.constants;

import com.ctre.phoenix6.CANBus;

import frc.robot.Constants.CANBuses;
import frc.robot.utils.TalonConfig;

/**
 * The arm constants
 * <br>
 * </br>
 * This class is divided to couple of diffrent static class for each part in the
 * arm
 */
public class ArmConstants {

    /** The name of the Subsystem */
    public static final String NAME = "Arm";

    /**
     * All the constnats of the calculation
     */
    public static class CalculationsConstants {
        public static final double BASE_HEIGHT = 0.86;
        public static final double ARM_1_LEN = 0.53;
        public static final double ARM_2_LEN = 0.32;
    }

    /**
     * All the constants of the arm angle motor
     */
    public static class ArmAngleMotorConstants {
        /* all the main configs of the motor */
        public static final int ID = 20;
        public static final CANBus CAN_BUS = CANBuses.ARM_CAN_BUS;
        public static final String NAME = "Arm Angle Motor";

        /* the pid and ff constants of the motor */
        public static final double KP = 22;
        public static final double KI = 1.0;
        public static final double KD = 0.75;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KG = 0;

        /* the motion magic constants of the motor */
        public static final double MOTION_MAGIC_VELOCITY = 1.5;
        public static final double MOTION_MAGIC_ACCELERATION = 3;
        public static final double MOTION_MAGIC_JERK = 6;

        /* the channel of the limit switch of the arm angle motor */
        public static final int LIMIT_SWITCH_CHANNEL = 0;

        /* the basic configues of the motor */
        public static final boolean IS_BRAKE = true;
        public static final boolean IS_INVERTED = false;
        public static final double GEAR_RATIO = 36.0 * (105.0 / 16.0);

        /* the ramp time of the motor */
        public static final double RAMP_TIME = 0.5;

        /*
         * all the angles of the motor
         * base -> where the limit switch
         * back limit -> the minimum angle
         * forward limit -> the maximum angle
         */
        public static final double BASE_ANGLE = Math.toRadians(33.7);
        public static final double BACK_LIMIT = Math.toRadians(33.7);
        public static final double FWD_LIMIT = 2.904541015625;

        /* The config of the motors based on the constants above */
        public static final TalonConfig CONFIG = new TalonConfig(ID, CAN_BUS, NAME)
                .withPID(KP, KI, KD, KS, KV, KA, KG)
                .withMotionMagic(MOTION_MAGIC_VELOCITY, MOTION_MAGIC_ACCELERATION, MOTION_MAGIC_JERK)
                .withBrake(IS_BRAKE)
                .withInvert(IS_INVERTED)
                .withMotorRatio(GEAR_RATIO).withRadiansMotor()
                .withRampTime(RAMP_TIME);
    }

    /**
     * the constants for the gripper going automaticly to a specific angle when the
     * arm is less than a specific angle
     */
    public static class GripperAngleStarting {
        public static final double WHEN_MOVING_GRIPPER = Math.toRadians(38);
        public static final double ANGLE_TO_GRIPPER = 3.75;
    }

    /** All the constants for the gripper angle motor */
    public static class GripperAngleMotorConstants {
        /* All the main configs of the motor */
        public static final int ID = 21;
        public static final CANBus CAN_BUS = CANBuses.ARM_CAN_BUS;
        public static final String NAME = "Gripper Angle Motor";

        /* the pid and ff of the motor */
        public static final double KP = 9.5;
        public static final double KI = 1.9;
        public static final double KD = 0.25;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KG = 0;

        /* the motion magic constants of the motor */
        public static final double MOTION_MAGIC_VELOCITY = 0;
        public static final double MOTION_MAGIC_ACCELERATION = 0;
        public static final double MOTION_MAGIC_JERK = 0;

        /* the channel of the absolute sensor */
        public static final int ABSOLUTE_SENSOR_CHANNEL = 1;

        /* all the basic configs of the motor */
        public static final boolean IS_BRAKE = true;
        public static final boolean IS_INVERTED = false;
        public static final double GEAR_RATIO = 36.0 * (47.0 / 20.0);

        /* the ramp time of the motor */
        public static final double RAMP_TIME = 0.5;

        /*
         * all the angles of the motor
         * base -> where the limit switch
         * back limit -> the minimum angle
         * forward limit -> the maximum angle
         */
        public static final double ENCODER_BASE_ANGLE = 3.1763315220836814 - 1.5*Math.PI;
        public static final double BACK_LIMIT = 3.7;
        public static final double FWD_LIMIT = 5.4;

        /* The config of the motor based on the constants above */
        public static final TalonConfig CONFIG = new TalonConfig(ID, CAN_BUS, NAME)
                .withPID(KP, KI, KD, KS, KV, KA, KG)
                .withPID1(KP*3, KI, KD, KS, KV, KA, KG)
                .withMotionMagic(MOTION_MAGIC_VELOCITY, MOTION_MAGIC_ACCELERATION, MOTION_MAGIC_JERK)
                .withBrake(IS_BRAKE)
                .withInvert(IS_INVERTED)
                .withMotorRatio(GEAR_RATIO).withRadiansMotor()
                .withRampTime(RAMP_TIME);
    }

    /** All the constants of the calibration */
    public static class CalibrationConstants {
        public static final double ARM_ANGLE_POWER = -0.3;
        public static final double ARM_ANGLE_START_POWER = 0.3;
        public static final double TIME_TO_CHANGE_POWER = 0.75;
    }

    /** The max errors of the arm and gripper angles */
    public static class MaxErrors {
        public static final double ARM_ANGLE_UP_ERROR = 0.02;
        public static final double ARM_ANGLE_DOWN_ERROR = 0.04;
        public static final double GRIPPER_ANGLE_UP_ERROR = 0.017;
        public static final double GRIPPER_ANGLE_DOWN_ERROR = 0.03;
    }

    /** all the constants angles */
    // public static class ANGLES_SADNA {
    //     public static final Pair<Double, Double> L1 = new Pair<Double, Double>(Math.toRadians(37.3), 4.6);
    //     public static final Pair<Double, Double> L2 = new Pair<Double, Double>(1.8, 4.4);
    //     public static final Pair<Double, Double> L3 = new Pair<Double, Double>(2.6, 4.63);
    //     public static final Pair<Double, Double> CORAL_STATION = new Pair<Double, Double>(1.58, 5.3);
    //     public static final Pair<Double, Double> CLIMB = new Pair<Double, Double>(2.8 ,5.45);
    //     public static final Pair<Double, Double> STARTING = new Pair<Double, Double>(Math.toRadians(33.7), 3.64);
    //     public static class Algae{
            
    //         public static final Pair<Double, Double> PRE_ALGAE_TOP = new Pair<Double, Double>(1.8, 3.7);
    //         public static final Pair<Double, Double> AFTER_ALGAE_TOP = new Pair<Double, Double>(2.5, 4.6);

    //         public static final Pair<Double, Double> PRE_ALGAE_BOTTOM = new Pair<Double, Double>(2.4, 2.5);
    //         public static final Pair<Double, Double> AFTER_ALGAE_BOTTOM = new Pair<Double, Double>(1.6, 2.5);
    //     }
    // }

    /** the arm angle states */
    public static enum ARM_ANGLE_STATES {
        L1(Math.toRadians(37.3), 4.6),
        L2(1.8, 4.4),
        L3(2.64208984375, 4.729228859906724),
        PRE_ALGAE_BOTTOM(2.5, 2.5),
        PRE_ALGAE_TOP(1.8, 3.7),
        AFTER_ALGAE_BOTTOM(1.6, 2.5),
        AFTER_ALGAE_TOP(2.5, 4.6),
        CORAL_STATION(1.6, 5.3),
        CLIMB(2.766 ,5.4),
        TESTING(0,0),
        STARTING(Math.toRadians(33.7), 3.64),
        IDLE(0,0);

        public final double armAngle;
        public final double gripperAngle;

        ARM_ANGLE_STATES(double armAngle, double gripperAngle) {
            this.armAngle = armAngle;
            this.gripperAngle = gripperAngle;
        }
    }

    /* COMP ANGLES
     * 
     * L1(Math.toRadians(37.3), 4.6),
     * L2(1.8, 4.4),
     * L3(2.56, 4.6),
     * PRE_ALGAE_BOTTOM(2.5, 2.5),
     * PRE_ALGAE_TOP(1.8, 3.7),
     * AFTER_ALGAE_BOTTOM(1.6, 2.5),
     * AFTER_ALGAE_TOP(2.5, 4.6),
     * CORAL_STATION(1.6, 5.3),
     * CLIMB(2.8 ,5.45),
     * TESTING(0,0),
     * STARTING(Math.toRadians(33.7), 3.64),
     * IDLE(0,0);
     */
}
