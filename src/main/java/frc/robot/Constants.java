package frc.robot;

public class Constants {
    public static final class CanIDs {
        public static final int SHOOTER_LEFT_MOTOR = 30;
        public static final int SHOOTER_RIGHT_MOTOR = 31;
        public static final int SHOOTER_HOOD = 38;
        public static final int SHOOTER_FEEDER = 39;

        public static final int CLIMBER_MOTOR = 32;

        public static final int TRANSFER_LEAD_INDEX = 33;
        public static final int TRANSFER_FOLLOW_INDEX = 34;
        public static final int TRANSFER_BELT = 35;

        public static final int INTAKE_ARM = 36;
        public static final int INTAKE_WHEELS = 37;
    }

    public static final double HOOD_GEAR_RATIO = 5.0 * 5.0 * (40 / 15); // 5:1 x 5:1 gearbox + 40/15 tooth gear ratio

    public static enum HoodPositions {
        STOW(0),
        NEAR_SHOT(0),
        FAR_SHOT(0),
        FEEDING(0);

        private final double angleDegrees;

        HoodPositions(double angleDegrees) {
            this.angleDegrees = angleDegrees;
        }

        public double toRotations() {
            return (angleDegrees / 360.0) * HOOD_GEAR_RATIO;
        }
    }

    // # of rotations for final climb
    // NEED TO TUNE!!!
    // never mind we don't
    public static final int CLIMBER_FINAL_POSITION = 10;
}