// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.orangeoverdrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import com.orangeoverdrive.generated.drivetrain.DrivetrainTelemetry;
import com.orangeoverdrive.subsystems.DrivetrainSubsystem;
import com.orangeoverdrive.subsystems.IntakeSubsystem;
import com.orangeoverdrive.subsystems.ShooterSubsystem;
import com.orangeoverdrive.subsystems.TransferSubsystem;

public class RobotContainer {
    private final DrivetrainTelemetry logger = new DrivetrainTelemetry(DrivetrainSubsystem.MAX_SPEED_METERS_PER_SECOND);

    // Subsystems
    public final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    public final ShooterSubsystem shooter;
    public final IntakeSubsystem intake;
    public final TransferSubsystem transfer;

    private final Controllers controllers = new Controllers();
    private final Autos autos;

    public RobotContainer() {
        shooter = new ShooterSubsystem();
        intake = new IntakeSubsystem();
        transfer = new TransferSubsystem();
        autos = new Autos(drivetrain, shooter, transfer);

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

        boolean allMotorsConnected = intakeConnected
                && shooterConnected
                && transferConnected
                && drivetrainConnected;

        SmartDashboard.putBoolean("Intake Motors Connected", intakeConnected);
        SmartDashboard.putBoolean("Shooter Motors Connected", shooterConnected);
        SmartDashboard.putBoolean("Transfer Motors Connected", transferConnected);
        SmartDashboard.putBoolean("Drivetrain Motors Connected", drivetrainConnected);
        SmartDashboard.putBoolean("allMotorsConnected", allMotorsConnected);
        SmartDashboard.putBoolean("Motor Status", allMotorsConnected);
    }

    public Command getAutonomousCommand() {
        return autos.getAutonomousCommand();
    }
}
