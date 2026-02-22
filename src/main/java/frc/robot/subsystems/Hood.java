package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.HoodConstants;

public class Hood extends SubsystemBase {
    private SparkMax motor;

    private RelativeEncoder encoder;

    private SparkClosedLoopController controller;

    private Trigger resetTrigger;
    private Trigger timerTrigger;

    private Timer resetTimer;

    private double currentTargetDegrees;

    public Hood() {
        motor = new SparkMax(HoodConstants.kMotorCANID, MotorType.kBrushless);

        motor.configure(
            HoodConstants.kConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );   

        encoder = motor.getEncoder();

        controller = motor.getClosedLoopController();

        resetTimer = new Timer();
        resetTimer.reset();

        resetTrigger = new Trigger(() -> (motor.getOutputCurrent() > HoodConstants.kAmpsToTriggerPositionReset));
        resetTrigger.onTrue(new InstantCommand(resetTimer::start));
        resetTrigger.onFalse(new InstantCommand(() -> {
            resetTimer.stop();
            resetTimer.reset();
        }));

        timerTrigger = new Trigger(() -> resetTimer.hasElapsed(HoodConstants.kTimeAboveThresholdToReset));
        timerTrigger.onTrue(new InstantCommand(() -> {
            encoder.setPosition(0);
            resetTimer.reset();
        }));

        currentTargetDegrees = HoodConstants.kStartupAngle;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Hood/OutputCurrent", motor.getOutputCurrent());
        Logger.recordOutput("Hood/CurrentTarget", currentTargetDegrees);
        Logger.recordOutput("Hood/CurrentAngle", encoder.getPosition());
        Logger.recordOutput("Hood/AtSetpoint", controller.isAtSetpoint());
    }

    public Command trackToAngle(DoubleSupplier degreeAngleSupplier) {
        return run(() -> {
            currentTargetDegrees = degreeAngleSupplier.getAsDouble();

            controller.setSetpoint(currentTargetDegrees, ControlType.kPosition);
        });
    }

    /**
     * An automated form of resetting the hood position sensing.
     * 
     * Run down at the full manual speed (note that this is affected by the 
     * kMaxManualSpeedMultiplier constant) until the timer trigger becomes true
     * (i.e. the output current has been above the threshold (kAmpsToTriggerPositionReset) 
     * for reset for the amount of specified by kTimeAboveThresholdToReset). Once
     * that returns true, the motor is stopped until the timer trigger switches to false
     * (i.e. it has reset the position automatically, because that's how it's configured, 
     * and resets the timer to 0, which makes the timer trigger false)
     * 
     * @return A complete Command structure that performs the specified action
     */
    public Command automatedRezero() {
        return manualSpeed(() -> -1)
            .until(timerTrigger)
            .andThen(
                stop().until(timerTrigger.negate())
        );
    }

    /**
     * An alternate form of {@link #automatedRezero()} that doesn't rely on the triggers
     * to reset the hood position to zero. Note that this method doesn't have any time limiting
     * factor to it, as soon as the current goes above the threshold specified by 
     * kAmpsToTriggerPositionReset the encoder position will be set to zero
     * 
     * @return A complete Command structure that performs the specified action
     */
    public Command automatedRezeroNoTimer() {
        return manualSpeed(() -> -1)
            .until(() -> motor.getOutputCurrent() >= HoodConstants.kAmpsToTriggerPositionReset)
            .andThen(new InstantCommand(() -> encoder.setPosition(0)));
    }

    public Command manualSpeed(DoubleSupplier speed) {
        currentTargetDegrees = 0;

        return run(() -> {
            motor.set(speed.getAsDouble() * HoodConstants.kMaxManualSpeedMultiplier);
        });
    }

    public Command stop() {
        return manualSpeed(() -> 0);
    }

    public double getTargetDegrees() {
        return currentTargetDegrees;
    }
}
