package frc.robot.subsystems;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class HoodSubsystem implements Subsystem{
    private final SparkMax hood;
    private final double hoodGearRatio = ((40.0 / 15.0) * 25.0); // 66.67 rotations of motor is one rotation of axle

    // constructor
    public HoodSubsystem() {
        SmartDashboard.putNumber("Hood Position", 0);

        hood = new SparkMax(Constants.CanIDs.SHOOTER_HOOD, MotorType.kBrushless);
    }

    public void setHoodPosition() {
        hood.getEncoder().setPosition(SmartDashboard.getNumber("Hood Position", 0) / 360.0 * hoodGearRatio);
    }

    public void changeHoodPosition(double hoodValue) {
        hood.getEncoder().setPosition(hood.getEncoder().getPosition() + (hoodValue / 360.0 * hoodGearRatio));
    }

    public void setLimelightHoodPosition(double angle) {
        hood.getEncoder().setPosition(angle / 360.0 * hoodGearRatio);
        
    }
}
