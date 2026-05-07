package com.orangeoverdrive;

import com.orangeoverdrive.subsystems.DrivetrainSubsystem;
import com.orangeoverdrive.subsystems.ShooterSubsystem;
import com.orangeoverdrive.subsystems.TransferSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class Autos {
  private final DrivetrainSubsystem drivetrain;
  private final ShooterSubsystem shooter;
  private final TransferSubsystem transfer;

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public Autos(
      DrivetrainSubsystem drivetrain,
      ShooterSubsystem shooter,
      TransferSubsystem transfer) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.transfer = transfer;

    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          drivetrain::getPose,
          drivetrain::resetPose,
          drivetrain::getChassisSpeeds,
          (speeds, feedforwards) -> drivetrain.applyPathSpeeds(
              speeds,
              feedforwards.robotRelativeForcesXNewtons(),
              feedforwards.robotRelativeForcesYNewtons()),
          new PPHolonomicDriveController(
              new PIDConstants(10, 0, 0),
              new PIDConstants(7, 0, 0)),
          config,
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          drivetrain);
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder",
          ex.getStackTrace());
    }

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
        drivetrain.applyRequest(drivetrain.buildRobotCentricRequest(-0.25, 0, 0)).withTimeout(2),
        drivetrain.applyRequestOnce(drivetrain.buildRobotCentricRequest(0, 0, 0)),
        shooter.shootSequence(transfer).withTimeout(3),
        Commands.waitSeconds(1),
        shooter.shootSequence(transfer).withTimeout(3),
        Commands.waitSeconds(1),
        shooter.shootSequence(transfer).withTimeout(3));
  }
}
