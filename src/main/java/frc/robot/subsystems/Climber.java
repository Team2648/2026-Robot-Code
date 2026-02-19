package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.ClimberConstants.ClimberPositions;

public class Climber extends SubsystemBase {
    private SparkMax motor;

    private RelativeEncoder encoder;

    private SparkClosedLoopController controller;

    private ClimberPositions targetPosition;

    public Climber() {
        motor = new SparkMax(ClimberConstants.kMotorCANID, MotorType.kBrushless);

        motor.configure(
            ClimberConstants.kMotorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        encoder = motor.getEncoder();

        controller = motor.getClosedLoopController();

        targetPosition = null;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Climber/TargetPositionMeters", targetPosition == null ? -1 : targetPosition.getPositionMeters());
        Logger.recordOutput("Climber/CurrentPositionMeters", encoder.getPosition());
        Logger.recordOutput("Climber/AtSetpoint", controller.isAtSetpoint());
    }

    public Command maintainPosition(ClimberPositions position) {
        targetPosition = position;

        return run(() -> {
            controller.setSetpoint(
                position.getPositionMeters(), 
                ControlType.kPosition
            );
        });
    }

    public Command manualSpeed(DoubleSupplier speed) {
        targetPosition = null;

        return run(() -> {
            motor.set(speed.getAsDouble());
        });
    }

    public Command stop() {
        return manualSpeed(() -> 0);
    }
}
