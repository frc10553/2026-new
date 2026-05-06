// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.orangeoverdrive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import com.orangeoverdrive.generated.LimelightHelpers;
import com.orangeoverdrive.generated.drivetrain.Drivetrain;
import com.orangeoverdrive.generated.drivetrain.DrivetrainConstants;
import com.orangeoverdrive.generated.drivetrain.DrivetrainTelemetry;
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

    private final SwerveRequest.RobotCentric autoRobotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(0).withRotationalDeadband(0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Subsystems
    public final Drivetrain drivetrain = DrivetrainConstants.createDrivetrain();
    public final ShooterSubsystem shooter;
    public final IntakeSubsystem intake;
    public final TransferSubsystem transfer;
    // public final HoodSubsystem hood;

    private final Controllers controllers = new Controllers();

    public RobotContainer() {
        shooter = new ShooterSubsystem();
        intake = new IntakeSubsystem();
        transfer = new TransferSubsystem();
        // hood = new HoodSubsystem();

        autoChooser.setDefaultOption("Nothing", Commands.none());
        autoChooser.addOption("Shoot Auto", autoCommand());
        SmartDashboard.putData(autoChooser);

        controllers.configureDrive(drivetrain);
        controllers.configureAux(shooter, intake, transfer);
        drivetrain.registerTelemetry(logger::telemeterize);

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        SmartDashboard.putBoolean("Motor Status", true);

        CommandScheduler.getInstance().schedule(Commands.run(() -> {
            SmartDashboard.putBoolean("allMotorsConnected",
                    intake.isConnected() /* && hood.isConnected() */ && shooter.isConnected()
                            && transfer.isConnected() && drivetrain.isConnected());
        }));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private Command autoCommand() {
        return Commands.sequence(
                drivetrain.applyRequest(() -> autoRobotCentricDrive.withVelocityX(-0.25)
                        .withVelocityY(0).withRotationalRate(0)).withTimeout(2),
                drivetrain.runOnce(() -> drivetrain.setControl(
                        autoRobotCentricDrive.withVelocityX(0)
                                .withVelocityY(0).withRotationalRate(0))),
                shooter.shootSequence(transfer).withTimeout(3),
                Commands.waitSeconds(1),
                shooter.shootSequence(transfer).withTimeout(3),
                Commands.waitSeconds(1),
                shooter.shootSequence(transfer).withTimeout(3));
    }
}
