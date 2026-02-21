package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class IntakeSubsystem implements Subsystem {
    private final TalonFX armMotor;

    public IntakeSubsystem() {
        armMotor = new TalonFX(Constants.CanIDs.INTAKE_ARM);
        armMotor.setPosition(0);
    }

    public void armUp() {

    }

    public void armDown() {

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm position", armMotor.getRotorPosition().getValue().magnitude());
    }
}
