package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class ClimberSubsystem implements Subsystem {
    private final TalonFX motor;
    private final double defaultSpeed = 0.5;

    public static final double EXTEND_POSITION = 147.068; // don't have 
    public static final double RETRACT_POSITION = 91.192; // clamped down

    private boolean extended = false;

    public ClimberSubsystem() {
        motor = new TalonFX(Constants.CanIDs.CLIMBER_MOTOR);
        SmartDashboard.putNumber("Climber Speed", this.defaultSpeed);
        SmartDashboard.putNumber("Climber Rotations", motor.getRotorPosition().getValue().magnitude());
    }

    public double getEncoderPosition() {
        return motor.getRotorPosition().getValueAsDouble();
    }

    public Command climb() {
        // need to replace this with PID because this is super jank
        return Commands.either(
            // is already extended - dip down to pull up
            Commands.run(() -> motor.set(-defaultSpeed), this)
                .until(() -> getEncoderPosition() <= RETRACT_POSITION)
                .finallyDo(() -> { motor.set(0); extended = false; }),

            // not extended - go all the way up
            Commands.run(() -> motor.set(defaultSpeed), this)
                .until(() -> getEncoderPosition() >= EXTEND_POSITION)
                .finallyDo(() -> { motor.set(0); extended = true; }),

            () -> extended
        );
    }
}
