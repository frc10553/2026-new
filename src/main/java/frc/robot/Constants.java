package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class CanIDs {
        public static final int SHOOTER_LEFT_MOTOR = 30;
        public static final int SHOOTER_RIGHT_MOTOR = 31;
        // public static final int SHOOTER_HOOD = 38;
        public static final int SHOOTER_FEEDER = 39;

        public static final int CLIMBER_MOTOR = 32;

        public static final int TRANSFER_LEAD_INDEX = 33;
        public static final int TRANSFER_FOLLOW_INDEX = 34;
        public static final int TRANSFER_BELT = 35;

        public static final int INTAKE_ARM = 36;
        public static final int INTAKE_WHEELS = 37;



        
        //Arm Constants
        public static final int kMotorPort = 0;
        public static final int kEncoderAChannel = 0;
        public static final int kEncoderBChannel = 1;
        public static final int kJoystickPort = 0;

        public static final String kArmPositionKey = "ArmPosition";
        public static final String kArmPKey = "ArmP";

  // The P gain for the PID controller that drives this arm.
        public static final double kDefaultArmKp = 50.0;
        public static final double kDefaultArmSetpointDegrees = 75.0;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses)
        public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

        public static final double kArmReduction = 200;
        public static final double kArmMass = 8.0; // Kilograms
        public static final double kArmLength = Units.inchesToMeters(30);
        public static final double kMinAngleRads = Units.degreesToRadians(-75);
        public static final double kMaxAngleRads = Units.degreesToRadians(255);
    }

    // public static final double HOOD_GEAR_RATIO = 5.0 * 5.0 * (40 / 15); // 5:1 x 5:1 gearbox + 40/15 tooth gear ratio

    // public static final double INTAKE_RESTING_ROTATIONS = 0.6215;
    public static final double INTAKE_RESTING_ROTATIONS = -5.3208;

    // public static enum HoodPositions {
    //     STOW(0),
    //     NEAR_SHOT(30),
    //     FAR_SHOT(15),
    //     FEEDING(20);

    //     private final double angleDegrees;

    //     HoodPositions(double angleDegrees) {
    //         this.angleDegrees = angleDegrees;
    //     }

    //     public double toRotations() {
    //         return (angleDegrees / 360.0) * HOOD_GEAR_RATIO;
    //     }
    // }

    // # of rotations for final climb
    // NEED TO TUNE!!!
    // never mind we don't
    public static final int CLIMBER_FINAL_POSITION = 10;
}