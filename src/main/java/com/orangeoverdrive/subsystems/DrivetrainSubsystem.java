package com.orangeoverdrive.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Set;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.orangeoverdrive.generated.drivetrain.DrivetrainConstants;
import com.orangeoverdrive.generated.drivetrain.DrivetrainCore;
import com.orangeoverdrive.generated.drivetrain.DrivetrainTelemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class DrivetrainSubsystem extends SubsystemBase {
    public static final double MAX_SPEED_METERS_PER_SECOND = DrivetrainConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private static final double SLOW_MODE_DIVISOR = 4.25;
    private static final double DRIVE_DEADBAND_RATIO = 0.1;
    private static final double ROTATION_DEADBAND_RATIO = 0.1;

    private final DrivetrainCore drivetrainCore;
    private final DrivetrainTelemetry logger = new DrivetrainTelemetry(MAX_SPEED_METERS_PER_SECOND);

    private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final SwerveRequest.FieldCentric fieldCentricDriveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric robotCentricDriveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private final SwerveRequest.PointWheelsAt pointAtRequest = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

    private boolean slowMode = false;

    public DrivetrainSubsystem() {
        drivetrainCore = DrivetrainConstants.createDrivetrain();
        drivetrainCore.registerTelemetry(logger::telemeterize);
    }

    public Command applyRequest(SwerveRequest request) {
        return run(() -> drivetrainCore.setControl(request));
    }

    public Command applyRequestOnce(SwerveRequest request) {
        return runOnce(() -> drivetrainCore.setControl(request));
    }

    public Command updateFieldCentricDrive(
            DoubleSupplier forward,
            DoubleSupplier left,
            DoubleSupplier rotation) {
        return run(() -> drivetrainCore.setControl(fieldCentricDriveRequest
                .withDeadband(getDriveDeadband())
                .withRotationalDeadband(getRotationalDeadband())
                .withVelocityX(forward.getAsDouble() * getActiveMaxSpeed())
                .withVelocityY(left.getAsDouble() * getActiveMaxSpeed())
                .withRotationalRate(rotation.getAsDouble() * getMaxAngularRate())));
    }

    public Command updateRobotCentricDrive(
            DoubleSupplier forward,
            DoubleSupplier left,
            DoubleSupplier rotation) {
        return run(() -> drivetrainCore.setControl(robotCentricDriveRequest
                .withDeadband(getDriveDeadband())
                .withRotationalDeadband(getRotationalDeadband())
                .withVelocityX(forward.getAsDouble() * getActiveMaxSpeed())
                .withVelocityY(left.getAsDouble() * getActiveMaxSpeed())
                .withRotationalRate(rotation.getAsDouble() * getMaxAngularRate())));
    }

    public SwerveRequest buildRobotCentricRequest(double velocityX, double velocityY, double rotationalRate) {
        return new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withDeadband(0)
                .withRotationalDeadband(0)
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(rotationalRate);
    }

    public Pose2d getPose() {
        return drivetrainCore.getState().Pose;
    }

    public void resetPose(Pose2d pose) {
        drivetrainCore.resetPose(pose);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return drivetrainCore.getState().Speeds;
    }

    public void applyPathSpeeds(
            ChassisSpeeds speeds,
            double[] robotRelativeForcesXNewtons,
            double[] robotRelativeForcesYNewtons) {
        drivetrainCore.setControl(pathApplyRobotSpeeds
                .withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                .withWheelForceFeedforwardsX(robotRelativeForcesXNewtons)
                .withWheelForceFeedforwardsY(robotRelativeForcesYNewtons));
    }

    public Command pointWheelsAt(DoubleSupplier forward, DoubleSupplier left) {
        return run(() -> drivetrainCore.setControl(
                pointAtRequest.withModuleDirection(new Rotation2d(forward.getAsDouble(), left.getAsDouble()))));
    }

    public Command idle() {
        return applyRequest(idleRequest);
    }

    public Command brake() {
        return applyRequest(brakeRequest);
    }

    public Command sysIdQuasistatic(Direction direction) {
        return Commands.defer(() -> drivetrainCore.sysIdQuasistatic(direction), Set.of(this));
    }

    public Command sysIdDynamic(Direction direction) {
        return Commands.defer(() -> drivetrainCore.sysIdDynamic(direction), Set.of(this));
    }

    public void seedFieldCentric() {
        drivetrainCore.seedFieldCentric();
    }

    public void registerTelemetry(Consumer<SwerveDriveState> telemetryConsumer) {
        drivetrainCore.registerTelemetry(telemetryConsumer);
    }

    public boolean isConnected() {
        return drivetrainCore.isConnected();
    }

    public double getActiveMaxSpeed() {
        return MAX_SPEED_METERS_PER_SECOND / (slowMode ? SLOW_MODE_DIVISOR : 1);
    }

    public double getMaxAngularRate() {
        return maxAngularRate;
    }

    public double getDriveDeadband() {
        return getActiveMaxSpeed() * DRIVE_DEADBAND_RATIO;
    }

    public double getRotationalDeadband() {
        return maxAngularRate * ROTATION_DEADBAND_RATIO;
    }

    public void toggleSlowMode() {
        slowMode = !slowMode;
    }

    @Override
    public void periodic() {
        drivetrainCore.periodic();
    }
}
