package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class ClimberSubsystem implements Subsystem {
    private final TalonFX motor;
    private final double defaultSpeed = 0.9;

    public ClimberSubsystem() {
        motor = new TalonFX(Constants.CanIDs.CLIMBER_MOTOR);
        SmartDashboard.putNumber("Climber Speed", this.defaultSpeed);
    }

    public void startMotor() {
        motor.set(SmartDashboard.getNumber("Climber Speed", this.defaultSpeed));
    }

    public void stopMotor() {
        motor.set(0);
    }
}
