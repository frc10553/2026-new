// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.orangeoverdrive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import com.orangeoverdrive.generated.LimelightHelpers;
import com.orangeoverdrive.generated.drivetrain.Drivetrain;
import com.orangeoverdrive.generated.drivetrain.DrivetrainConstants;
import com.orangeoverdrive.generated.drivetrain.DrivetrainTelemetry;
import com.orangeoverdrive.subsystems.ClimberSubsystem;
import com.orangeoverdrive.subsystems.IntakeSubsystem;
import com.orangeoverdrive.subsystems.ShooterSubsystem;
import com.orangeoverdrive.subsystems.TransferSubsystem;
// import com.orangeoverdrive.subsystems.HoodSubsystem;
// import com.orangeoverdrive.Constants.HoodPositions;

public class RobotContainer {
    // Maximum speed
    // (gets divided by 4 for slow mode)
    private double MaxSpeed = DrivetrainConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final DrivetrainTelemetry logger = new DrivetrainTelemetry(MaxSpeed);

    // Subsystems
    public final Drivetrain drivetrain = DrivetrainConstants.createDrivetrain();
    public final ShooterSubsystem shooter;
    public final IntakeSubsystem intake;
    public final TransferSubsystem transfer;
    public final ClimberSubsystem climber;
    // public final HoodSubsystem hood;

    private final Controllers controllers = new Controllers();
    private final Autos autos;

    public RobotContainer() {
        shooter = new ShooterSubsystem();
        intake = new IntakeSubsystem();
        transfer = new TransferSubsystem();
        climber = new ClimberSubsystem();
        autos = new Autos(drivetrain, shooter, transfer);
        // hood = new HoodSubsystem();

        controllers.configureDrive(drivetrain);
        controllers.configureAux(shooter, intake, transfer);
        drivetrain.registerTelemetry(logger::telemeterize);

        // Warmup PathPlanner to avoid Java pauses
        autos.warmupPathPlanner();

        updateMotorStatus();

        CommandScheduler.getInstance().schedule(Commands.run(this::updateMotorStatus).ignoringDisable(true));
    }

    private void updateMotorStatus() {
        boolean intakeConnected = intake.isConnected();
        boolean shooterConnected = shooter.isConnected();
        boolean transferConnected = transfer.isConnected();
        boolean drivetrainConnected = drivetrain.isConnected();
        boolean climberConnected = climber.isConnected();

        boolean allMotorsConnected = intakeConnected
                && shooterConnected
                && transferConnected
                && drivetrainConnected
                && climberConnected;

        SmartDashboard.putBoolean("Intake Motors Connected", intakeConnected);
        SmartDashboard.putBoolean("Shooter Motors Connected", shooterConnected);
        SmartDashboard.putBoolean("Transfer Motors Connected", transferConnected);
        SmartDashboard.putBoolean("Drivetrain Motors Connected", drivetrainConnected);
        SmartDashboard.putBoolean("Climber Motor Connected", climberConnected);
        SmartDashboard.putBoolean("allMotorsConnected", allMotorsConnected);
        SmartDashboard.putBoolean("Motor Status", allMotorsConnected);
    }

    public Command getAutonomousCommand() {
        return autos.getAutonomousCommand();
    }
}
