package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeRollerConstants;

public class IntakeRoller extends SubsystemBase {
    private SparkMax leftMotor;
    private SparkMax rightMotor;

    public IntakeRoller() {
        leftMotor = new SparkMax(IntakeRollerConstants.kLeftMotorCANID, MotorType.kBrushless);
        rightMotor = new SparkMax(IntakeRollerConstants.kRightMotorCANID, MotorType.kBrushless);

        leftMotor.configure(
            IntakeRollerConstants.leftMotorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        rightMotor.configure(
            IntakeRollerConstants.rightMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public Command runIn() {
        return run(() -> {
            leftMotor.set(IntakeRollerConstants.kSpeed*0.8);
        });
    }

    public Command runOut() {
        return run(() -> {
            leftMotor.set(-IntakeRollerConstants.kSpeed);
        });
    }

    public Command stop() {
        return run(() -> {
            leftMotor.set(0);
        });
    }
    
}
