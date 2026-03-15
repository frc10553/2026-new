package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class IntakeSubsystem implements Subsystem {
    private final TalonFX armMotor;
    private final SparkMax intakeWheelMotor;

    public IntakeSubsystem() {
        armMotor = new TalonFX(Constants.CanIDs.INTAKE_ARM);
        intakeWheelMotor = new SparkMax(Constants.CanIDs.INTAKE_WHEELS, MotorType.kBrushless);
        SmartDashboard.putNumber("Arm position", 0);
    }

    public void armDeployOut() {
        armMotor.setVoltage(SmartDashboard.getNumber("Arm position", 0));
    }

    public void armStop() {
        armMotor.setVoltage(0.0);
    }

    public void armTakeIn() {
        armMotor.setPosition(0);
    }

    public void intakeStart(double volts) { 
        intakeWheelMotor.setVoltage(volts);
    }
    
    public void intakeStop() {
        intakeWheelMotor.setVoltage(0); 
    }

    public double getEncoderPosition() {
        return armMotor.getRotorPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intakePosition", armMotor.getRotorPosition().getValue().magnitude());
    }
}
