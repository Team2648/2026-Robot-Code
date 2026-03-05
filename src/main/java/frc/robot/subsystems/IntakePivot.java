package frc.robot.subsystems;

import java.util.Optional;
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
import frc.robot.constants.IntakePivotConstants;
import frc.robot.constants.IntakePivotConstants.IntakePivotPosition;

public class IntakePivot extends SubsystemBase {
    private SparkMax leftMotor;
    private SparkMax rightMotor;

    private RelativeEncoder encoder;

    private SparkClosedLoopController controller;

    private IntakePivotPosition currentTargetPosition;

    public IntakePivot() {
        leftMotor = new SparkMax(IntakePivotConstants.kLeftMotorCANID, MotorType.kBrushless);
        rightMotor = new SparkMax(IntakePivotConstants.kRightMotorCANID, MotorType.kBrushless);

        leftMotor.configure(
            IntakePivotConstants.KLeftMotorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        rightMotor.configure(
            IntakePivotConstants.kRightMotorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        controller = leftMotor.getClosedLoopController();

        encoder = leftMotor.getEncoder();
        encoder.setPosition(IntakePivotConstants.IntakePivotPosition.kUp.getPositionRadians());
    }

    @Override
    public void periodic() {
        Logger.recordOutput(
            "IntakePivot/TargetPosition", 
            currentTargetPosition == null ? -1 : currentTargetPosition.getPositionRadians());
        Logger.recordOutput("IntakePivot/CurrentPosition", encoder.getPosition());
        Logger.recordOutput("IntakePivot/AtSetpoint", controller.isAtSetpoint());
    }

    public Command maintainPosition(IntakePivotPosition position) {
        return run(() -> {
            currentTargetPosition = position;

            if(currentTargetPosition == null) {
                leftMotor.disable();
            } else {
                controller.setSetpoint(currentTargetPosition.getPositionRadians(), ControlType.kPosition);
            }
        });
    }

    public Command manualSpeed(DoubleSupplier speed) {
        return run(() -> {
            currentTargetPosition = null;
            
            leftMotor.set(speed.getAsDouble() * IntakePivotConstants.kMaxManualSpeedMultiplier);
            rightMotor.set(speed.getAsDouble() * IntakePivotConstants.kMaxManualSpeedMultiplier);
        });
    }

    public Command stop() {
        return manualSpeed(() -> 0);
    }

    public Optional<IntakePivotPosition> getCurrentTargetPosition() {
        return Optional.ofNullable(currentTargetPosition);
    }
}
