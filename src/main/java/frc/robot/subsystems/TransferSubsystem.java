package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class TransferSubsystem extends SubsystemBase {
    private final SparkMax beltMotor;
    private final double defaultSpeed = 9.0; // determined experimentally

    public TransferSubsystem() {
        beltMotor = new SparkMax(Constants.CanIDs.TRANSFER_BELT, MotorType.kBrushless);
    }

    public void startMotor() {
        beltMotor.setVoltage(defaultSpeed);
    }

    public void stopMotor() {
        beltMotor.setVoltage(0);
    }

    public void setVoltage(double volts) {
        beltMotor.setVoltage(volts);
    }
}
