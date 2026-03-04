package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class ClimberSubsystem implements Subsystem {
    private final TalonFX motor;
    private final double defaultSpeed = 0.9;

    public ClimberSubsystem() {
        motor = new TalonFX(Constants.CanIDs.CLIMBER_MOTOR);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 2.4;  // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0;    // no output for integrated error
        slot0Configs.kD = 0.1;  // A velocity of 1 rps results in 0.1 V output

        motor.getConfigurator().apply(slot0Configs);
        SmartDashboard.putNumber("Climber Speed", this.defaultSpeed);
    }

    public void climb() {
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        motor.setControl(m_request.withPosition(Constants.CLIMBER_FINAL_POSITION));
    }

    public void stopMotor() {
        motor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Rotations", motor.getRotorPosition().getValue().magnitude());
    }
}
