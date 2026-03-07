package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class ShooterSubsystem implements Subsystem {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final SparkMax feeder;
    private final SparkMax hood;
    private final double defaultSpeed = 0.9;

    // constructor
    public ShooterSubsystem() {
        SmartDashboard.putNumber("Hood Position", 0);
        SmartDashboard.putNumber("Shooter RPS", -100);
        SmartDashboard.putNumber("Shooter Arm P", 0.02);
        SmartDashboard.putNumber("Shooter Arm I", 0);
        SmartDashboard.putNumber("Shooter Arm D", 0);
        SmartDashboard.putNumber("Feeder Volts", 3);

        leftMotor = new TalonFX(Constants.CanIDs.SHOOTER_LEFT_MOTOR);
        rightMotor = new TalonFX(Constants.CanIDs.SHOOTER_RIGHT_MOTOR);
        feeder = new SparkMax(Constants.CanIDs.SHOOTER_FEEDER, MotorType.kBrushless);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0; // Add 0.1 V output to overcome static friction
        slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = SmartDashboard.getNumber("Shooter Arm P", 0); // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = SmartDashboard.getNumber("Shooter Arm I", 0); // no output for integrated error
        slot0Configs.kD = SmartDashboard.getNumber("Shooter Arm D", 0); // no output for error derivative

        leftMotor.getConfigurator().apply(slot0Configs);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Aligned));

        leftMotor.setControl(new CoastOut());

        hood = new SparkMax(Constants.CanIDs.SHOOTER_HOOD, MotorType.kBrushless);
    }

    public void startMotor() {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0; // Add 0.1 V output to overcome static friction
        slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = SmartDashboard.getNumber("Shooter Arm P", 0.02); // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = SmartDashboard.getNumber("Shooter Arm I", 0); // no output for integrated error
        slot0Configs.kD = SmartDashboard.getNumber("Shooter Arm D", 0); // no output for error derivative

        leftMotor.getConfigurator().apply(slot0Configs);

        final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
        leftMotor.setControl(m_request.withVelocity(SmartDashboard.getNumber("Shooter RPS", -100)));

        feeder.setVoltage(SmartDashboard.getNumber("Feeder Volts", 3));
    }

    public void hoodPosition(Constants.HoodPositions hoodPosition) {
        hood.getEncoder().setPosition(SmartDashboard.getNumber("Hood Position", 0));
    }

    public void stopMotor() {
        leftMotor.set(0);
        feeder.set(0);
    }
}
