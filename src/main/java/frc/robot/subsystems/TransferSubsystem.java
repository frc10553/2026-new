package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class TransferSubsystem implements Subsystem {
    private final TalonFX leadIndexMotor;
    private final TalonFX followIndexMotor;
    private final TalonFX beltMotor;
    private final double defaultSpeed = 0.9;

    public TransferSubsystem() {
        leadIndexMotor = new TalonFX(Constants.CanIDs.TRANSFER_LEAD_INDEX);
        followIndexMotor = new TalonFX(Constants.CanIDs.TRANSFER_FOLLOW_INDEX);
        beltMotor = new TalonFX(Constants.CanIDs.TRANSFER_BELT);
        SmartDashboard.putNumber("Transfer Speed", this.defaultSpeed);
    }
}
