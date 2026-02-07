package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {
    private TalonFX motor;

    private DutyCycleOut motorOutput;

    public Spindexer() {
        motor = new TalonFX(SpindexerConstants.kMotorCANID);

        motor.getConfigurator().apply(SpindexerConstants.kCurrentLimitConfig);
        motor.getConfigurator().apply(SpindexerConstants.kMotorConfig);

        motorOutput = new DutyCycleOut(0);
    }

    public Command spinToShooter() {
        return run(() -> {
            motor.setControl(motorOutput.withOutput(1));
        });
    }

    public Command spinToIntake() {
        return run(() -> {
            motor.setControl(motorOutput.withOutput(-1));
        });
    }

    public Command stop() {
        return run(() -> {
            motor.setControl(motorOutput.withOutput(0));
        });
    }
    
}
