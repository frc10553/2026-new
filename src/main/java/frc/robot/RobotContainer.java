// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

    // Maximum speed
    // (is divided by 4 for slow mode)
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private double SlowAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond);
    private boolean slowMode = false;
    private boolean robotCentric = false;

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
    // ^^^ for use later

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ShooterSubsystem shooter;
    public final IntakeSubsystem intake; 

    private final Telemetry logger = new Telemetry(MaxSpeed);
    
    public RobotContainer() {
        this.shooter = new ShooterSubsystem();
        intake = new IntakeSubsystem();
        configureBindings();
    }

    private void configureBindings() {
        // +X is forward
        // +Y is left
        if (robotCentric) {
            drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(controller1.getLeftY() * MaxSpeed)
                        .withVelocityY(controller1.getLeftX() * MaxSpeed)
                        .withRotationalRate(-controller1.getRightX() * MaxAngularRate)));
        } else {
            drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> fieldCentricDrive.withVelocityX(controller1.getLeftY() * MaxSpeed)
                        .withVelocityY(controller1.getLeftX() * MaxSpeed)
                        .withRotationalRate(-controller1.getRightX() * MaxAngularRate)));
        }

        controller1.leftTrigger().onTrue(Commands.runOnce(() -> {
            robotCentric = !robotCentric;
        }));

        // Idle motors when disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        controller1.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller1.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-controller1.getLeftY(), -controller1.getLeftX()))));

        // Process sysids
        controller1.back().and(controller1.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller1.back().and(controller1.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller1.start().and(controller1.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller1.start().and(controller1.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Re-center the field-centric heading
        controller1.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Slower drive toggle
        controller1.povDown().onTrue(Commands.runOnce(() -> {
            this.slowMode = !this.slowMode;
            this.MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / (this.slowMode ? 4 : 1);
        }));

        // Shooter
        controller2.rightBumper().onTrue(Commands.runOnce(() -> {
            System.out.println("shooter is processing");
            shooter.startMotor();
        }));

        controller2.rightBumper().onFalse(Commands.runOnce(() -> {
            System.out.println("not going");
            shooter.stopMotor();
        })); 

        //intake 

        controller2.leftBumper().onTrue(Commands.runOnce(() -> {
            System.out.println("hhh");
            intake.intakeStart();
        })); 

        controller2.leftBumper().onFalse(Commands.runOnce(() -> {
            intake.intakeStop();
        })); 

        controller2.x().onTrue(Commands.runOnce(() -> {
            intake.armDeployOut();
        })); 

        controller2.y().onTrue(Commands.runOnce(() -> {
            intake.armTakeIn();
        })); 

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
