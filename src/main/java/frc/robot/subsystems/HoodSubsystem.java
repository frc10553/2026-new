package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.HoodPositions;

public class HoodSubsystem implements Subsystem {
    private final SparkMax hood;
    private final SparkClosedLoopController closedLoop;

    private double targetRotations = 0.0;

    // revlib marked .configure for removal
    // but the docs don't have anything else as a replacement so yeah
    @SuppressWarnings("removal")
    public HoodSubsystem() {
        hood = new SparkMax(Constants.CanIDs.SHOOTER_HOOD, MotorType.kBrushless);
        closedLoop = hood.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
                .p(0.1)
                .i(0)
                .d(0);

        hood.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        hood.getEncoder().setPosition(0);
    }

    // PID to a setpoint
    public void setHoodPreset(HoodPositions preset) {
        closedLoop.setSetpoint(preset.toRotations(), ControlType.kPosition);
    }

    // set to arbitrary rotations (if needed)
    public void setHoodPosition(double rotations) {
        closedLoop.setSetpoint(rotations, ControlType.kPosition);
    }


    public void setVoltage(double voltage) {
        hood.setVoltage(voltage);
    }

    public void holdCurrentPosition() {
        targetRotations = hood.getEncoder().getPosition();
        closedLoop.setSetpoint(targetRotations, ControlType.kPosition);
    }

    // for smartdashboard
    public double getTargetRotations() {
        return targetRotations;
    }

    public double getEncoderPosition() {
        return hood.getEncoder().getPosition();
    }
}
