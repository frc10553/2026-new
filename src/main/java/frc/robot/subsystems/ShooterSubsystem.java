package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class ShooterSubsystem implements Subsystem {
    private final TalonFX motor;
    private final SparkMax hood;
    private final double defaultSpeed = 0.9;

    // constructor
    public ShooterSubsystem() {
        motor = new TalonFX(Constants.CanIDs.SHOOTER_LEFT_MOTOR);
        hood = new SparkMax(Constants.CanIDs.SHOOTER_HOOD, MotorType.kBrushless);
        SmartDashboard.putNumber("Shooter Speed", this.defaultSpeed);
    }

    public void startMotor() {
        motor.set(SmartDashboard.getNumber("Shooter Speed", this.defaultSpeed));
    }

    public void hoodPosition(Constants.HoodPositions hoodPosition) {

    }

    public void stopMotor() {
        motor.set(0);
    }
}
