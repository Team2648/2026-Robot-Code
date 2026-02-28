package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeRollerConstants;

public class IntakeRoller extends SubsystemBase {
    private SparkMax motor;

    public IntakeRoller() {
        motor = new SparkMax(IntakeRollerConstants.kMotorCANID, MotorType.kBrushless);

        motor.configure(
            IntakeRollerConstants.leftMotorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );
    }

    public Command runIn() {
        return run(() -> {
            motor.set(IntakeRollerConstants.kSpeed);
        });
    }

    public Command runOut() {
        return run(() -> {
            motor.set(-IntakeRollerConstants.kSpeed);
        });
    }

    public Command stop() {
        return run(() -> {
            motor.set(0);
        });
    }
    
}
