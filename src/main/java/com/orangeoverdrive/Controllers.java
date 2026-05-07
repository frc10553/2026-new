package com.orangeoverdrive;

import com.orangeoverdrive.subsystems.DrivetrainSubsystem;
import com.orangeoverdrive.subsystems.IntakeSubsystem;
import com.orangeoverdrive.subsystems.ShooterSubsystem;
import com.orangeoverdrive.subsystems.TransferSubsystem;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class Controllers {
  private boolean robotCentric = false;

  private final CommandXboxController driver;
  private final CommandXboxController aux;

  public Controllers() {
    driver = new CommandXboxController(0);
    aux = new CommandXboxController(1);
  }

  private void updateDriveDefaultCommand(DrivetrainSubsystem drivetrain) {
    if (robotCentric) {
      drivetrain.setDefaultCommand(
          drivetrain.updateRobotCentricDrive(
              () -> -driver.getLeftY(),
              () -> -driver.getLeftX(),
              () -> -driver.getRightX()));
    } else {
      drivetrain.setDefaultCommand(
          drivetrain.updateFieldCentricDrive(
              () -> -driver.getLeftY(),
              () -> -driver.getLeftX(),
              () -> -driver.getRightX()));
    }
  }

  public void configureDrive(DrivetrainSubsystem drivetrain) {
    // +X is forward
    // +Y is left
    // This is the driving control
    updateDriveDefaultCommand(drivetrain);

    driver.povUp().onTrue(Commands.runOnce(() -> {
      robotCentric = !robotCentric;
      updateDriveDefaultCommand(drivetrain);
    }));

    // Idle motors when disabled
    RobotModeTriggers.disabled().whileTrue(drivetrain.idle().ignoringDisable(true));

    driver.leftTrigger().whileTrue(drivetrain.brake());
    driver.b().whileTrue(drivetrain.pointWheelsAt(() -> -driver.getLeftY(), () -> -driver.getLeftX()));

    // Process sysids
    driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Re-center the field-centric heading
    driver.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // Slower drive toggle
    driver.leftBumper().onTrue(Commands.runOnce(drivetrain::toggleSlowMode, drivetrain));
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
