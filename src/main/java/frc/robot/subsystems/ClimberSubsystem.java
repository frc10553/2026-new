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
        SmartDashboard.putBoolean("invertClimber", false);
    }

    public void startClimbing() {
        motor.set(SmartDashboard.getNumber("Climber Speed", defaultSpeed));
    }

    public void stopClimbing() {
        motor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Rotations", motor.getRotorPosition().getValue().magnitude());
    }
}
