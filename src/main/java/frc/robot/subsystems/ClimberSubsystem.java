package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class ClimberSubsystem implements Subsystem {
    private final TalonFX motor;
    private final double defaultSpeed = 0.5;

    public ClimberSubsystem() {
        motor = new TalonFX(Constants.CanIDs.CLIMBER_MOTOR);
        SmartDashboard.putNumber("Climber Speed", this.defaultSpeed);
        SmartDashboard.putNumber("Climber Rotations", motor.getRotorPosition().getValue().magnitude());
    }

    public void setClimbPosition(double pos) {
        motor.setPosition(pos);
    }

    public void startClimbing() {
        // runs periodically

        if (motor.getRotorPosition().getValue().magnitude() < 147.0) {
            motor.set(SmartDashboard.getNumber("Climber Speed", defaultSpeed));
        } else {
            motor.setVoltage(0.0);
        }
    }

    public void stopClimbing() {
        motor.setPosition(91.192);
    }

    public double getEncoderPosition() {
        return motor.getRotorPosition().getValueAsDouble();
    }
}
