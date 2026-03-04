package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0; // Add 0.1 V output to overcome static friction
        slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = SmartDashboard.getNumber("Shooter Arm P", 0); // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = SmartDashboard.getNumber("Shooter Arm I", 0); // no output for integrated error
        slot0Configs.kD = SmartDashboard.getNumber("Shooter Arm D", 0); // no output for error derivative

        motor.getConfigurator().apply(slot0Configs);

         hood = new SparkMax(Constants.CanIDs.SHOOTER_HOOD, MotorType.kBrushless);
        SmartDashboard.putNumber("Shooter Speed", this.defaultSpeed);
    }

    public void startMotor() {
        // motor.set(SmartDashboard.getNumber("Shooter Speed", this.defaultSpeed));

        final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
        motor.setControl(m_request.withVelocity(SmartDashboard.getNumber("Shooter RPS", 100)));
    }

    public void hoodPosition(Constants.HoodPositions hoodPosition) {

    }

    public void stopMotor() {
        motor.set(0);
    }
}
