package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {
    private TalonFX spindexerMotor;

    private SparkMax feederMotor;

    private DutyCycleOut spindexerMotorOutput;

    public Spindexer() {
        spindexerMotor = new TalonFX(SpindexerConstants.kSpindexerMotorCANID);

        feederMotor = new SparkMax(SpindexerConstants.kFeederMotorCANID, MotorType.kBrushless);

        spindexerMotor.getConfigurator().apply(SpindexerConstants.kSpindexerCurrentLimitConfig);
        spindexerMotor.getConfigurator().apply(SpindexerConstants.kSpindexerMotorConfig);

        feederMotor.configure(
            SpindexerConstants.kFeederConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        spindexerMotorOutput = new DutyCycleOut(0);
    }

    public Command spinToShooter() {
        return run(() -> {
            spindexerMotor.setControl(spindexerMotorOutput.withOutput(1));
            feederMotor.set(1);
        });
    }

    public Command spinToIntake() {
        return run(() -> {
            spindexerMotor.setControl(spindexerMotorOutput.withOutput(-1));
            feederMotor.set(-1);
        });
    }

    public Command stop() {
        return run(() -> {
            spindexerMotor.setControl(spindexerMotorOutput.withOutput(0));
            feederMotor.set(0);
        });
    }
    
}
