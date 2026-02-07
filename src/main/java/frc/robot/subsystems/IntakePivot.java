package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
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

    private AbsoluteEncoder encoder;

    private SparkClosedLoopController controller;

    private IntakePivotPosition currentTargetPosition;

    public IntakePivot() {
        leftMotor = new SparkMax(IntakePivotConstants.kLeftMotorCANID, MotorType.kBrushless);
        rightMotor = new SparkMax(IntakePivotConstants.kRightMotorCANID, MotorType.kBrushless);

        leftMotor.configure(
            IntakePivotConstants.leftMotorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        rightMotor.configure(
            IntakePivotConstants.rightMotorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        controller = leftMotor.getClosedLoopController();

        encoder = leftMotor.getAbsoluteEncoder();
    }

    @Override
    public void periodic() {
        if(currentTargetPosition == null) {
            leftMotor.disable();
            rightMotor.disable();
        } else {
            controller.setSetpoint(currentTargetPosition.getPositionRadians(), ControlType.kPosition);
        }

        Logger.recordOutput(
            "IntakePivot/TargetPosition", 
            currentTargetPosition == null ? -1 : currentTargetPosition.getPositionRadians());
        Logger.recordOutput("IntakePivot/CurrentPosition", encoder.getPosition());
        Logger.recordOutput("IntakePivot/AtSetpoint", controller.isAtSetpoint());
    }

    public Command setTargetPosition(IntakePivotPosition position) {
        return runOnce(() -> {
            currentTargetPosition = position;
        });
    }

    public Command stop() {
        return runOnce(() -> {
            currentTargetPosition = null;
        });
    }

    public Optional<IntakePivotPosition> getCurrentTargetPosition() {
        return Optional.of(currentTargetPosition);
    }
}
