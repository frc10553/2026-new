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
        armMotor.setPosition(0);
        intakeWheelMotor = new SparkMax(Constants.CanIDs.INTAKE_WHEELS, MotorType.kBrushless);
        SmartDashboard.putNumber("Arm position", 0);
        SmartDashboard.putNumber("Intake voltage", 4);
    }

    public void armUp() {

    }

    public void armDown() {

    }

    public void intakeStart() { 
        // 3 VOLTS IS LIKE PERFECT???
        intakeWheelMotor.setVoltage(SmartDashboard.getNumber("Intake voltage", 3));
    }
    
    public void intakeStop() {
        intakeWheelMotor.setVoltage(0); 
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm position", armMotor.getRotorPosition().getValue().magnitude());
    }
}
