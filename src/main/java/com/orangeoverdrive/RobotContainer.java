// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.orangeoverdrive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.orangeoverdrive.generated.LimelightHelpers;
import com.orangeoverdrive.generated.drivetrain.Drivetrain;
import com.orangeoverdrive.generated.drivetrain.Constants;
import com.orangeoverdrive.generated.drivetrain.Telemetry;
import com.orangeoverdrive.subsystems.IntakeSubsystem;
import com.orangeoverdrive.subsystems.ShooterSubsystem;
import com.orangeoverdrive.subsystems.ClimberSubsystem;
import com.orangeoverdrive.subsystems.TransferSubsystem;
// import com.orangeoverdrive.subsystems.HoodSubsystem;
// import com.orangeoverdrive.Constants.HoodPositions;

public class RobotContainer {

    // Maximum speed
    // (is divided by 4 for slow mode)
    private double MaxSpeed = Constants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private boolean slowMode = false;
    private boolean robotCentric = false;

    private boolean limelightToggle = true;

    // Basic targeting data
    private double tx = LimelightHelpers.getTX(""); // Horizontal offset from crosshair to target in degrees
    private double ty = LimelightHelpers.getTY(""); // Vertical offset from crosshair to target in degrees
    private double ta = LimelightHelpers.getTA(""); // Target area (0% to 100% of image)
    private boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

    // Platform drive methods
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            // 10% deadband
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            // 10% deadband
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric autoRobotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(0).withRotationalDeadband(0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final CommandXboxController controller1 = new CommandXboxController(0);
    private final CommandXboxController controller2 = new CommandXboxController(1);

    SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Subsystems
    public final Drivetrain drivetrain = Constants.createDrivetrain();
    public final ShooterSubsystem shooter;
    public final IntakeSubsystem intake;
    public final TransferSubsystem transfer;
    // public final HoodSubsystem hood;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* Path follower */
    // private final SendableChooser<Command> autoChooser;

    private final Command autoCommand() {
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

    public RobotContainer() {
        shooter = new ShooterSubsystem();
        intake = new IntakeSubsystem();
        transfer = new TransferSubsystem();
        // hood = new HoodSubsystem();

        // autoChooser = AutoBuilder.buildAutoChooser("Tests");
        // SmartDashboard.putData("Auto Mode", autoChooser);

        // SendableChooser<Command> autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Nothing", Commands.none());
        autoChooser.addOption("Shoot Auto", autoCommand());

        SmartDashboard.putData(autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        // SmartDashboard.putNumber("Hood Angle Adjustment", 0.25);
        SmartDashboard.putNumber("Climb Position", 0.0);
        // SmartDashboard.putNumber("Hood Position", 0);
        // SmartDashboard.getNumber("Far Hood Position", 0);

        // false = down
        // true = up
        SmartDashboard.putBoolean("Manual Climber Direction", false);

        SmartDashboard.putBoolean("Motor Status", true);

        CommandScheduler.getInstance().schedule(Commands.run(() -> {
            SmartDashboard.putBoolean("allMotorsConnected",
                    intake.isConnected() /* && hood.isConnected() */ && shooter.isConnected()
                            && transfer.isConnected() && drivetrain.isConnected());
        }));
    }

    private void configureBindings() {
        // +X is forward
        // +Y is left
        // This is the driving control
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> fieldCentricDrive
                .withVelocityX(-controller1.getLeftY() * MaxSpeed).withVelocityY(-controller1.getLeftX() * MaxSpeed)
                .withRotationalRate(-controller1.getRightX() * MaxAngularRate)));

        controller1.povUp().onTrue(Commands.runOnce(() -> {
            robotCentric = !robotCentric;
            if (robotCentric) {
                drivetrain.setDefaultCommand(
                        drivetrain
                                .applyRequest(() -> robotCentricDrive.withVelocityX(-controller1.getLeftY() * MaxSpeed)
                                        .withVelocityY(-controller1.getLeftX() * MaxSpeed)
                                        .withRotationalRate(-controller1.getRightX() * MaxAngularRate)));
            } else {
                drivetrain.setDefaultCommand(
                        drivetrain
                                .applyRequest(() -> fieldCentricDrive.withVelocityX(-controller1.getLeftY() * MaxSpeed)
                                        .withVelocityY(-controller1.getLeftX() * MaxSpeed)
                                        .withRotationalRate(-controller1.getRightX() * MaxAngularRate)));
            }
        }));

        // Idle motors when disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        controller1.leftTrigger().whileTrue(drivetrain.applyRequest(() -> brake));
        controller1.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-controller1.getLeftY(), -controller1.getLeftX()))));

        // Process sysids
        controller1.back().and(controller1.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller1.back().and(controller1.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller1.start().and(controller1.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller1.start().and(controller1.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Re-center the field-centric heading
        controller1.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Slower drive toggle
        controller1.leftBumper().onTrue(Commands.runOnce(() -> {
            this.slowMode = !this.slowMode;
            this.MaxSpeed = Constants.kSpeedAt12Volts.in(MetersPerSecond) / (this.slowMode ? 4.25 : 1);
        }));

        controller2.rightBumper()
                .whileTrue(shooter.shootSequence(transfer/* , intake */));

        // Feeding mode
        controller2.rightTrigger()
                .whileTrue(shooter.feedingSequence(transfer/* ,intake */));

        controller2.a().onTrue(Commands.runOnce(() -> {
            shooter.startFeeder(false);
        }));
        controller2.a().onFalse(Commands.runOnce(() -> {
            shooter.stopMotor();
        }));

        controller2.b().onTrue(Commands.runOnce(() -> {
            shooter.startMotor(false);
        }));
        controller2.b().onFalse(Commands.runOnce(() -> {
            shooter.stopMotor();
        }));
        // hood shooting not as far
        // controller2.a().onTrue(Commands.runOnce(() -> {
        // hood.setHoodPreset(HoodPositions.NEAR_SHOT);
        // }));

        // // hood shooting far
        // controller2.b().onTrue(Commands.runOnce(() -> {
        // hood.setHoodPreset(HoodPositions.FAR_SHOT);
        // }));

        // Manual hood adjust — hold D-pad for voltage, release to lock position with
        // PID
        // controller2.povUp().onTrue(Commands.runOnce(() -> {
        // hood.setVoltage(0.75);
        // }));
        // controller2.povUp().onFalse(Commands.runOnce(() -> {
        // hood.holdCurrentPosition();
        // }));

        // controller2.povDown().onTrue(Commands.runOnce(() -> {
        // hood.setVoltage(-0.75);
        // }));
        // controller2.povDown().onFalse(Commands.runOnce(() -> {
        // hood.holdCurrentPosition();
        // }));

        controller2.leftBumper().whileTrue(intake.runIntake(2.5));
        controller2.leftTrigger().whileTrue(intake.runIntake(4.5));
        controller2.y().whileTrue(intake.runIntake(11));

        controller2.povLeft().whileTrue(Commands.startEnd(() -> {
            intake.setVoltage(-3);
            transfer.setVoltage(-3);
        }, () -> {
            intake.setVoltage(0);
            transfer.setVoltage(0);
        }));

        // controller1.rightBumper().whileTrue(intake.agitateIntake(transfer));
        // controller1.rightBumper().onFalse(Commands.runOnce(() -> {
        // intake.moveWithPID(Constants.INTAKE_RESTING_ROTATIONS);
        // }));

        // Deploy arm (in theory)
        // controller2.x().onTrue(Commands.runOnce(intake::armDeployOut, intake));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
