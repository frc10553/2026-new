package com.orangeoverdrive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.orangeoverdrive.generated.drivetrain.Drivetrain;
import com.orangeoverdrive.generated.drivetrain.DrivetrainConstants;
import com.orangeoverdrive.subsystems.IntakeSubsystem;
import com.orangeoverdrive.subsystems.ShooterSubsystem;
import com.orangeoverdrive.subsystems.TransferSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class Controllers {
  private double MaxSpeed = DrivetrainConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private boolean slowMode = false;
  private boolean robotCentric = false;

  private final CommandXboxController driver;
  private final CommandXboxController aux;

  private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public Controllers() {
    driver = new CommandXboxController(0);
    aux = new CommandXboxController(1);
  }

  private void applyDriveDefaultCommand(Drivetrain drivetrain) {
    if (robotCentric) {
      drivetrain.setDefaultCommand(
          drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(-driver.getLeftY() * MaxSpeed)
              .withVelocityY(-driver.getLeftX() * MaxSpeed)
              .withRotationalRate(-driver.getRightX() * MaxAngularRate)));
    } else {
      drivetrain.setDefaultCommand(
          drivetrain.applyRequest(() -> fieldCentricDrive.withVelocityX(-driver.getLeftY() * MaxSpeed)
              .withVelocityY(-driver.getLeftX() * MaxSpeed)
              .withRotationalRate(-driver.getRightX() * MaxAngularRate)));
    }
  }

  public void configureDrive(Drivetrain drivetrain) {
    // +X is forward
    // +Y is left
    // This is the driving control
    applyDriveDefaultCommand(drivetrain);

    driver.povUp().onTrue(Commands.runOnce(() -> {
      robotCentric = !robotCentric;
      applyDriveDefaultCommand(drivetrain);
    }));

    // Idle motors when disabled
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    driver.leftTrigger().whileTrue(drivetrain.applyRequest(() -> brake));
    driver.b().whileTrue(drivetrain.applyRequest(
        () -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // Process sysids
    driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Re-center the field-centric heading
    driver.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // Slower drive toggle
    driver.leftBumper().onTrue(Commands.runOnce(() -> {
      slowMode = !slowMode;
      MaxSpeed = DrivetrainConstants.kSpeedAt12Volts.in(MetersPerSecond) / (slowMode ? 4.25 : 1);
    }));
  }

  public void configureAux(
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      TransferSubsystem transfer) {
    aux.rightBumper()
        .whileTrue(shooter.shootSequence(transfer /* , intake */));

    // Feeding mode
    aux.rightTrigger()
        .whileTrue(shooter.feedingSequence(transfer/* ,intake */));

    aux.a().onTrue(Commands.runOnce(() -> {
      shooter.startFeeder(false);
    }));
    aux.a().onFalse(Commands.runOnce(() -> {
      shooter.stopMotor();
    }));

    aux.b().onTrue(Commands.runOnce(() -> {
      shooter.startMotor(false);
    }));
    aux.b().onFalse(Commands.runOnce(() -> {
      shooter.stopMotor();
    }));
    // hood shooting not as far
    // aux.a().onTrue(Commands.runOnce(() -> {
    // hood.setHoodPreset(HoodPositions.NEAR_SHOT);
    // }));

    // // hood shooting far
    // aux.b().onTrue(Commands.runOnce(() -> {
    // hood.setHoodPreset(HoodPositions.FAR_SHOT);
    // }));

    // Manual hood adjust — hold D-pad for voltage, release to lock position with
    // PID
    // aux.povUp().onTrue(Commands.runOnce(() -> {
    // hood.setVoltage(0.75);
    // }));
    // aux.povUp().onFalse(Commands.runOnce(() -> {
    // hood.holdCurrentPosition();
    // }));

    // aux.povDown().onTrue(Commands.runOnce(() -> {
    // hood.setVoltage(-0.75);
    // }));
    // aux.povDown().onFalse(Commands.runOnce(() -> {
    // hood.holdCurrentPosition();
    // }));

    aux.leftBumper().whileTrue(intake.runIntake(2.5));
    aux.leftTrigger().whileTrue(intake.runIntake(4.5));
    aux.y().whileTrue(intake.runIntake(11));

    aux.povLeft().whileTrue(Commands.startEnd(() -> {
      intake.setVoltage(-3);
      transfer.setVoltage(-3);
    }, () -> {
      intake.setVoltage(0);
      transfer.setVoltage(0);
    }));

    // driver.rightBumper().whileTrue(intake.agitateIntake(transfer));
    // driver.rightBumper().onFalse(Commands.runOnce(() -> {
    // intake.moveWithPID(Constants.INTAKE_RESTING_ROTATIONS);
    // }));

    // Deploy arm (in theory)
    // aux.x().onTrue(Commands.runOnce(intake::armDeployOut, intake));
  }
}
