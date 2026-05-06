package com.orangeoverdrive.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.orangeoverdrive.Constants.HoodPositions;
import com.orangeoverdrive.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final SparkMax feeder;
    //private final double defaultSpeed = 0.9;
    // private static double kMaxVelocity = 1.75;
    // private static double kMaxAcceleration = 0.75;
    // private static final double kP = 1.3;
    // private static final double kI = 0.0;
    // private static double kD = 0.7;
    // private double targetRPM = 0;

    // private final PIDController leftController = new PIDController(kP, kI, kD);


    // constructor
    public ShooterSubsystem() {

        leftMotor = new TalonFX(Constants.CanIDs.SHOOTER_LEFT_MOTOR);
        rightMotor = new TalonFX(Constants.CanIDs.SHOOTER_RIGHT_MOTOR);
        feeder = new SparkMax(Constants.CanIDs.SHOOTER_FEEDER, MotorType.kBrushless);
        
        // leftController.setTolerance(100);
        
        var motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        var invertedMotorOutputConfigs = new MotorOutputConfigs();
        invertedMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        invertedMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0;
        slot0Configs.kV = 0.2;
        //3
        slot0Configs.kP = 3;
        //0
        slot0Configs.kI = 0;
        //0.35
        slot0Configs.kD = 0.35;

        var currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = 60;
        currentLimits.StatorCurrentLimit = 100;

        var configs = new TalonFXConfiguration();
        configs.Slot0 = slot0Configs;
        configs.MotorOutput = motorOutputConfigs;
        configs.CurrentLimits = currentLimits;

        rightMotor.getConfigurator().apply(configs);
        leftMotor.getConfigurator().apply(configs.withMotorOutput(invertedMotorOutputConfigs));
        
        leftMotor.setControl(new CoastOut());
        rightMotor.setControl(new CoastOut());


        SmartDashboard.putNumber("activeShooterVelocity", rightMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter RPS", 60);
    }

    public void startMotor(boolean feeding) {
        if(feeding){
            rightMotor.setControl(new VelocityVoltage(-100));
            leftMotor.setControl(new VelocityVoltage(-100));

        }else{
            leftMotor.setControl(new VelocityVoltage(-SmartDashboard.getNumber("Shooter RPS", 60)));
            rightMotor.setControl(new VelocityVoltage(-SmartDashboard.getNumber("Shooter RPS", 60)));
        }
    }

    // public void setTargetRPM(double rpm){
    //     targetRPM = rpm;
    // }

    // public void stop(){
    //     targetRPM = 0;
    // }

    public double getCurrentRPM(){
        // TODO get actual encoder value
        return 0;
    }

    // public boolean atSpeed(){
    //     return Math.abs(getCurrentRPM()-targetRPM)<100;
    // }

    public void startFeeder(boolean crossFielding) {
        feeder.setVoltage(crossFielding ? -7 : -8);
    }

    public void stopFeeder() {
        feeder.setVoltage(0);
    }

    public void stopMotor() {
        leftMotor.setControl(new CoastOut());
        rightMotor.setControl(new CoastOut());
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
        SmartDashboard.putNumber("activeShooterVelocityRight", Math.abs(rightMotor.getVelocity().getValueAsDouble()));
        SmartDashboard.putNumber("activeShooterVelocityLeft", Math.abs(leftMotor.getVelocity().getValueAsDouble()));
        // double currentRPM = getCurrentRPM();
        // double output = leftController.calculate(currentRPM, targetRPM);
        // output = Math.max(-1, Math.min(1, output));
        // System.out.println(targetRPM + currentRPM + output);
    }

    public boolean isConnected() {
        SparkBase.Faults feederFaults = feeder.getFaults();
        if (feederFaults.can || feederFaults.motorType) {
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
