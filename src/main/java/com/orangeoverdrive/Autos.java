package com.orangeoverdrive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.orangeoverdrive.generated.drivetrain.Drivetrain;
import com.orangeoverdrive.subsystems.ShooterSubsystem;
import com.orangeoverdrive.subsystems.TransferSubsystem;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class Autos {
  private final Drivetrain drivetrain;
  private final ShooterSubsystem shooter;
  private final TransferSubsystem transfer;

  private final SwerveRequest.RobotCentric autoRobotCentricDrive = new SwerveRequest.RobotCentric()
      .withDeadband(0).withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public Autos(
      Drivetrain drivetrain,
      ShooterSubsystem shooter,
      TransferSubsystem transfer) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.transfer = transfer;

    autoChooser.setDefaultOption("Nothing", Commands.none());
    autoChooser.addOption("Shoot Auto", autoCommand());
    SmartDashboard.putData(autoChooser);
  }

  public void warmupPathPlanner() {
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
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
