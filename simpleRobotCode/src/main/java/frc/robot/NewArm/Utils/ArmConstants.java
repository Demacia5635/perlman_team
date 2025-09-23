package frc.robot.NewArm.Utils;

public class ArmConstants {
    public static enum STATEHEIGHT{
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

        STATEHEIGHT(double armAngle, double gripperAngle) {
            this.armAngle = armAngle;
            this.gripperAngle = gripperAngle;
        }
    }

    class Height{
        public static final double BASE_HEIGHT = 0.86;
        public static final double ARM_1_LEN = 0.53;
        public static final double ARM_2_LEN = 0.32;
    }

    public static final double BASE_ANGLE = Math.toRadians(33.7);
}
