package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.HoodPositions;

public class HoodSubsystem implements Subsystem{
    private final SparkMax hood;

    // constructor
    public HoodSubsystem() {
        hood = new SparkMax(Constants.CanIDs.SHOOTER_HOOD, MotorType.kBrushless);
    }

    // pos is in angles
    public void setHoodPosition(double pos) {
        hood.getEncoder().setPosition(pos);
    }

    public void changeHoodPosition(double hoodValue) {
        hood.setVoltage(hoodValue);
    }

    public void setHoodPreset(HoodPositions preset) {
        hood.getEncoder().setPosition(preset.toRotations());
    }

    public void setLimelightHoodPosition(double angle) {
        hood.getEncoder().setPosition(angle);
    }

    public double getEncoderPosition() {
        return hood.getEncoder().getPosition();
    }
}
