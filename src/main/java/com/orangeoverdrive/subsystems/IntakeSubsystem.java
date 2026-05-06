package com.orangeoverdrive.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.orangeoverdrive.Constants;

public class IntakeSubsystem extends SubsystemBase {
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

    public void moveWithPID(double rotations) {
        final PositionVoltage pidRequest = new PositionVoltage(0).withSlot(0);
        armMotor.setControl(pidRequest.withPosition(rotations));
    }

    public void armDeployOut() {
        moveWithPID(Constants.INTAKE_RESTING_ROTATIONS);
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

    public Command agitateIntake(TransferSubsystem transfer) {
        return Commands.sequence(
                Commands.runOnce(() -> armMotor.setVoltage(1.25)),
                Commands.runOnce(() -> transfer.setVoltage(9)),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> moveWithPID(Constants.INTAKE_RESTING_ROTATIONS)),
                Commands.runOnce(() -> transfer.setVoltage(7)),
                Commands.waitSeconds(0.5),

                Commands.runOnce(() -> armMotor.setVoltage(1.25)),
                Commands.runOnce(() -> transfer.setVoltage(9)),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> moveWithPID(Constants.INTAKE_RESTING_ROTATIONS)),
                Commands.runOnce(() -> transfer.setVoltage(7)),
                Commands.waitSeconds(0.5),

                Commands.runOnce(() -> armMotor.setVoltage(1.25)),
                Commands.runOnce(() -> transfer.setVoltage(9)),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> moveWithPID(Constants.INTAKE_RESTING_ROTATIONS)),
                Commands.runOnce(() -> transfer.setVoltage(7)),
                Commands.waitSeconds(0.5));
    }

    public double getEncoderPosition() {
        return armMotor.getRotorPosition().getValueAsDouble();
    }

    public void setVoltage(double volts) {
        intakeWheelMotor.setVoltage(volts);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Position", armMotor.getRotorPosition().getValue().magnitude());
    }

    public boolean isConnected() {
        SparkBase.Faults intakeFaults = intakeWheelMotor.getFaults();
        if (intakeFaults.can || intakeFaults.motorType) {
            return false;
        }

        if (!armMotor.isConnected() || !armMotor.isAlive()) {
            return false;
        }

        return true;
    }
}
