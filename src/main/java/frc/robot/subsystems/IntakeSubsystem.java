package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class IntakeSubsystem implements Subsystem {
    private final TalonFX armMotor;
    private final SparkMax intakeWheelMotor;

    public IntakeSubsystem() {
        armMotor = new TalonFX(Constants.CanIDs.INTAKE_ARM);
        intakeWheelMotor = new SparkMax(Constants.CanIDs.INTAKE_WHEELS, MotorType.kBrushless);
        SmartDashboard.putNumber("Arm Position", 0);

        var pid = new Slot0Configs();
        pid.kP = 0.43; // An error of 1 rotation results in 2.4 V output
        pid.kI = 0.06; // no output for integrated error
        pid.kD = 0; // A velocity of 1 rps results in 0.1 V output

        armMotor.getConfigurator().apply(pid);
    }

    public void armDeployOut() {
        // armMotor.setVoltage(SmartDashboard.getNumber("Arm Position", 0));
        // create a position closed-loop request, voltage output, slot 0 configs
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

        armMotor.setControl(m_request.withPosition(0.6215));
    }

    public void armStop() {
        armMotor.setVoltage(0.0);
    }

    public void armTakeIn() {
        armMotor.setPosition(0);
    }

    public Command runIntake(double volts) {
        return Commands.startEnd(
                () -> intakeWheelMotor.setVoltage(volts),
                () -> intakeWheelMotor.setVoltage(0));
    }

    public Command agitateIntake() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
                armMotor.setControl(m_request.withPosition(0.8));
            }),
            Commands.waitSeconds(0.2),
            Commands.runOnce(() -> {
                final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
                armMotor.setControl(m_request.withPosition(0.6215));
            }),
            Commands.runOnce(() -> {
                final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
                armMotor.setControl(m_request.withPosition(0.8));
            }),
            Commands.waitSeconds(0.2),
            Commands.runOnce(() -> {
                final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
                armMotor.setControl(m_request.withPosition(0.6215));
            }),
            Commands.runOnce(() -> {
                final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
                armMotor.setControl(m_request.withPosition(0.8));
            }),
            Commands.waitSeconds(0.2),
            Commands.runOnce(() -> {
                final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
                armMotor.setControl(m_request.withPosition(0.6215));
            })
        );
    }

    public double getEncoderPosition() {
        return armMotor.getRotorPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Position", armMotor.getRotorPosition().getValue().magnitude());
    }
}
