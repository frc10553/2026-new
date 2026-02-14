// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity
    private double SlowAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                       // max angular velocity
    private boolean practice = true;
    private boolean robotCentric;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController controller1 = new CommandXboxController(0);
    // private final CommandXboxController controller2 = new
    // CommandXboxController(1);
    // ^^^ for use later

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ShooterSubsystem shooter;

    public RobotContainer() {
        this.shooter = new ShooterSubsystem();
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(controller1.getLeftY() * MaxSpeed) // Drive forward
                                                                                                     // with Y (forward)
                        .withVelocityY(controller1.getLeftX() * MaxSpeed) // Drive left with X (left)
                        .withRotationalRate(-controller1.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                       // negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        controller1.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller1.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-controller1.getLeftY(), -controller1.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller1.back().and(controller1.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller1.back().and(controller1.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller1.start().and(controller1.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller1.start().and(controller1.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        controller1.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // shooter
        controller1.povUp().onTrue(Commands.runOnce(() -> {
            System.out.println("shooter is processing");
            shooter.startMotor();
        }));

        controller1.povUp().onFalse(Commands.runOnce(() -> {
            System.out.println("not going");
            shooter.stopMotor();
        }));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
