package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.HoodPositions;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final SparkMax feeder;
    private final double defaultSpeed = 0.9;

    // constructor
    public ShooterSubsystem() {

        leftMotor = new TalonFX(Constants.CanIDs.SHOOTER_LEFT_MOTOR);
        rightMotor = new TalonFX(Constants.CanIDs.SHOOTER_RIGHT_MOTOR);
        feeder = new SparkMax(Constants.CanIDs.SHOOTER_FEEDER, MotorType.kBrushless);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0;
        slot0Configs.kV = 0;
        slot0Configs.kP = 2;
        slot0Configs.kI = 1.93;
        slot0Configs.kD = 0.18;

        var currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = 60;
        currentLimits.StatorCurrentLimit = 100;

        rightMotor.getConfigurator().apply(slot0Configs);
        rightMotor.getConfigurator().apply(currentLimits);

        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), MotorAlignmentValue.Aligned));

        rightMotor.setControl(new CoastOut());

        SmartDashboard.putNumber("activeShooterVelocity", rightMotor.getVelocity().getValueAsDouble());
    }

    public void startMotor(boolean feeding) {
        final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
        rightMotor.setControl(m_request.withVelocity(feeding ? -100 : -SmartDashboard.getNumber("Shooter RPS", 80)));
    }

    public void startFeeder(boolean crossFielding) {
        feeder.setVoltage(crossFielding ? -7 : -8);
    }

    public void stopFeeder() {
        feeder.setVoltage(0);
    }

    public void stopMotor() {
        rightMotor.set(0);
        feeder.set(0);
    }

    public Command shootSequence(TransferSubsystem transfer/*, IntakeSubsystem intake*/) {
        return Commands.parallel(
            Commands.sequence(
                Commands.runOnce(() -> startMotor(false), this),
                // wait for motors to get up to speed
                Commands.waitSeconds(1.5),
                Commands.runOnce(() -> {
                    startFeeder(false);
                })),
            
            // Commands.sequence(Commands.waitSeconds(2.0), Commands.repeatingSequence(
            //     Commands.runOnce(() -> intake.setVoltage(2.5)), 
            //     Commands.waitSeconds(1), 
            //     Commands.runOnce(() -> intake.setVoltage(4.5)),
            //     Commands.waitSeconds(1)
            // )),

            Commands.sequence(Commands.waitSeconds(2.0), Commands.repeatingSequence(
                Commands.runOnce(() -> transfer.setVoltage(9)), 
                Commands.waitSeconds(1), 
                Commands.runOnce(() -> transfer.setVoltage(5)),
                Commands.waitSeconds(1)
            )))

                // still stops the motors even if this exits before the waitSeconds finishes
                // (from what I understand)
                .finallyDo(() -> {
                    stopMotor();
                    stopFeeder();
                    transfer.stopMotor();
                    //intake.setVoltage(0);
                });
    }

    public Command feedingSequence(TransferSubsystem transfer/*, IntakeSubsystem intake*/) {
        return Commands.parallel(
            Commands.sequence(
                Commands.runOnce(() -> startMotor(true), this),
                // wait for motors to get up to speed
                Commands.waitSeconds(1.5),
                Commands.runOnce(() -> {
                    startFeeder(true);
                })),
            
            // Commands.sequence(Commands.waitSeconds(2.0), Commands.repeatingSequence(
            //     Commands.runOnce(() -> intake.setVoltage(2.5)), 
            //     Commands.waitSeconds(1), 
            //     Commands.runOnce(() -> intake.setVoltage(4.5)),
            //     Commands.waitSeconds(1)
            // )),

            Commands.sequence(Commands.waitSeconds(2.0), Commands.repeatingSequence(
                Commands.runOnce(() -> transfer.setVoltage(9)), 
                Commands.waitSeconds(1), 
                Commands.runOnce(() -> transfer.setVoltage(5)),
                Commands.waitSeconds(1)
            )))

                // still stops the motors even if this exits before the waitSeconds finishes
                // (from what I understand)
                .finallyDo(() -> {
                    stopMotor();
                    stopFeeder();
                    transfer.stopMotor();
                    //intake.setVoltage(0);
                });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("activeShooterVelocity", rightMotor.getVelocity().getValueAsDouble());
    }

    public boolean isConnected() {
        SparkBase.Faults feederFaults = feeder.getFaults();
        if (!feederFaults.can && !feederFaults.motorType) {
            return false;
        }

        if (!rightMotor.isConnected() || !leftMotor.isConnected()) {
            return false;
        }

        if (!rightMotor.isAlive() || !leftMotor.isAlive()) {
            return false;
        }

        return true;
    }
}
