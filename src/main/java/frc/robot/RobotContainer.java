// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

import frc.robot.generated.TunerConstants;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.Constants.HoodPositions;

public class RobotContainer {

    // Maximum speed
    // (is divided by 4 for slow mode)
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
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

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final CommandXboxController controller1 = new CommandXboxController(0);
    private final CommandXboxController controller2 = new CommandXboxController(1);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ShooterSubsystem shooter;
    public final IntakeSubsystem intake;
    public final TransferSubsystem transfer;
    public final ClimberSubsystem climber;
    public final HoodSubsystem hood;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* Path follower */
    // private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        this.shooter = new ShooterSubsystem();
        intake = new IntakeSubsystem();
        transfer = new TransferSubsystem();
        climber = new ClimberSubsystem();
        hood = new HoodSubsystem();

        // autoChooser = AutoBuilder.buildAutoChooser("Tests");
        // SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        SmartDashboard.putNumber("Hood Angle Adjustment", 0.25);
        SmartDashboard.putNumber("Climb Position", 0.0);
        SmartDashboard.putNumber("Hood Position", 0);
        SmartDashboard.getNumber("Far Hood Position", 0);

        // false = down
        // true = up
        SmartDashboard.putBoolean("Manual Climber Direction", false);
    }

    private void configureBindings() {
        // +X is forward
        // +Y is left
        // This is the driving control
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> fieldCentricDrive
                .withVelocityX(-controller1.getLeftY() * MaxSpeed).withVelocityY(-controller1.getLeftX() * MaxSpeed)
                .withRotationalRate(-controller1.getRightX() * MaxAngularRate)));

        controller1.leftTrigger().onTrue(Commands.runOnce(() -> {
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

        controller1.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller1.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-controller1.getLeftY(), -controller1.getLeftX()))));

        // Process sysids
        controller1.back().and(controller1.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller1.back().and(controller1.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller1.start().and(controller1.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller1.start().and(controller1.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Re-center the field-centric heading
        controller1.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Slower drive toggle
        controller1.povDown().onTrue(Commands.runOnce(() -> {
            this.slowMode = !this.slowMode;
            this.MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / (this.slowMode ? 4 : 1);
        }));

        // Shooter
        controller2.rightTrigger()
                .whileTrue(shooter.shootSequence(transfer));

        // Feeding mode
        controller2.rightBumper()
                .whileTrue(shooter.feedingSequence(transfer, hood));

        // Hood
        // hood.setDefaultCommand(Commands.runOnce(() -> {
        //     if (limelightToggle) {

        //     }
        // }, hood));

        climber.setDefaultCommand(Commands.runOnce(() -> {
            SmartDashboard.putNumber("Climber Rotations", climber.getEncoderPosition());
        }, climber));

        hood.setDefaultCommand(Commands.runOnce(() -> {
            SmartDashboard.putNumber("Hood Rotations", hood.getEncoderPosition());
            SmartDashboard.putNumber("Hood Target", hood.getTargetRotations());
        }, hood));

        // hood shooting not as far
        controller2.a().onTrue(Commands.runOnce(() -> {
            hood.setHoodPreset(HoodPositions.NEAR_SHOT);
        }));

        // hood shooting far
        controller2.b().onTrue(Commands.runOnce(() -> {
            hood.setHoodPreset(HoodPositions.FAR_SHOT);
        }));

        // Manual hood adjust — hold D-pad for voltage, release to lock position with
        // PID
        controller2.povUp().onTrue(Commands.runOnce(() -> {
            hood.setVoltage(0.75);
        }));
        controller2.povUp().onFalse(Commands.runOnce(() -> {
            hood.holdCurrentPosition();
        }));

        controller2.povDown().onTrue(Commands.runOnce(() -> {
            hood.setVoltage(-0.75);
        }));
        controller2.povDown().onFalse(Commands.runOnce(() -> {
            hood.holdCurrentPosition();
        }));

        // Intake
        intake.setDefaultCommand(Commands.runOnce(() -> {
            SmartDashboard.putNumber("Intake Rotations", intake.getEncoderPosition());
        }, intake));

        controller2.leftBumper().whileTrue(intake.runIntake(2.5));
        controller2.leftTrigger().whileTrue(intake.runIntake(4));
        controller2.leftStick().whileTrue(intake.runIntake(11));

        controller2.povLeft().whileTrue(Commands.startEnd(() -> {
            intake.setVoltage(-3);
            transfer.setVoltage(-3);
        }, () -> {
            intake.setVoltage(0);
            transfer.setVoltage(0);
        }));

        controller1.povRight().whileTrue(intake.agitateIntake(transfer));
        controller1.povRight().onFalse(Commands.runOnce(() -> {
            intake.moveWithPID(Constants.INTAKE_RESTING_ROTATIONS);
        }));


        // Deploy arm (in theory)
        controller2.x().onTrue(Commands.runOnce(intake::armDeployOut, intake));

        /// Climber
        // Press once to go all the way up, press again to dip down
        controller2.povRight().onTrue(climber.climb());

        // Manually move climber
        controller2.y()
                .onTrue(Commands.startEnd(
                        () -> climber.setVoltage(
                                SmartDashboard.getBoolean("Manual Climber Direction", false /* down */) ? -5 : 5),
                        () -> climber.setVoltage(0), climber));

        controller2.y().onTrue(Commands.runOnce(() -> climber
                .setVoltage(SmartDashboard.getBoolean("Manual Climber Direction", false /* down */) ? -5 : 5)));

        controller2.y().onFalse(Commands.runOnce(() -> climber.setVoltage(0)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // public Command getAutonomousCommand() {
    // /* Run the path selected from the auto chooser */
    //// return autoChooser.getSelected();
    // }
}
