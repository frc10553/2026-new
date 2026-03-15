package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class HoodSubsystem implements Subsystem{
    private final SparkMax hood;
    private final double hoodGearRatio = ((40.0 / 15.0) * 25.0); // 66.67 rotations of motor is one rotation of axle

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

    public void setLimelightHoodPosition(double angle) {
        hood.getEncoder().setPosition(angle);
        
    }

    public double getEncoderPosition() {
        return hood.getEncoder().getPosition();
    }
}
