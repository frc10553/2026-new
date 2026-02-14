package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ShooterSubsystem implements Subsystem {
    private final TalonFX motor;
    private final double defaultSpeed = 0.9;

    // constructor
    public ShooterSubsystem() {
        motor = new TalonFX(30);
        SmartDashboard.putNumber("Shooter Speed", this.defaultSpeed);
    }

    public void startMotor() {
        motor.set(SmartDashboard.getNumber("Shooter Speed", this.defaultSpeed));
    }

    public void stopMotor() {
        motor.set(0);
    } 
}
