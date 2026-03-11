package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class TransferSubsystem implements Subsystem {
    private final SparkMax beltMotor;
    private final double defaultSpeed = 0.9;

    public TransferSubsystem() {
        beltMotor = new SparkMax(Constants.CanIDs.TRANSFER_BELT, MotorType.kBrushless);
        SmartDashboard.putNumber("Transfer Speed", this.defaultSpeed);
    }

    public void startMotor() {
        beltMotor.setVoltage(SmartDashboard.getNumber("Transfer Speed", defaultSpeed));
    }

    public void stopMotor() {
        beltMotor.setVoltage(0);
    }
    
}
