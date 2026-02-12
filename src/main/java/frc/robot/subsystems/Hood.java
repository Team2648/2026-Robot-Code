package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HoodConstants;

public class Hood extends SubsystemBase {
    private SparkMax motor;

    private AbsoluteEncoder encoder;

    private SparkClosedLoopController controller;

    private double currentTargetRadians;

    public Hood() {
        motor = new SparkMax(HoodConstants.kMotorCANID, MotorType.kBrushless);

        encoder = motor.getAbsoluteEncoder();

        controller = motor.getClosedLoopController();

        currentTargetRadians = HoodConstants.kStartupAngle;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Hood/CurrentTarget", currentTargetRadians);
        Logger.recordOutput("Hood/CurrentAngle", encoder.getPosition());
        Logger.recordOutput("Hood/AtSetpoint", controller.isAtSetpoint());
    }

    public Command trackToAngle(DoubleSupplier radianAngleSupplier) {
        return run(() -> {
            currentTargetRadians = radianAngleSupplier.getAsDouble();

            controller.setSetpoint(currentTargetRadians, ControlType.kPosition);
        });
    }

    public Command manualSpeed(DoubleSupplier speed) {
        currentTargetRadians = 0;

        return run(() -> {
            motor.set(speed.getAsDouble() * HoodConstants.kMaxManualSpeedMultiplier);
        });
    }

    public Command stop() {
        return manualSpeed(() -> 0);
    }

    public double getTargetRadians() {
        return currentTargetRadians;
    }
}
