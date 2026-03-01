package frc.robot.subsystems;

import java.util.Optional;
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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.ShooterSpeeds;

public class Shooter extends SubsystemBase {
    private SparkMax leftMotor;
    private SparkMax rightMotor;

    private AbsoluteEncoder leftEncoder;
    private AbsoluteEncoder rightEncoder;

    private RelativeEncoder rightRelative;

    private SparkClosedLoopController leftClosedLoopController;
    private SparkClosedLoopController rightClosedLoopController;

    private ShooterSpeeds targetSpeeds;

    public Shooter() {
        leftMotor = new SparkMax(ShooterConstants.kLeftShooterMotorCANID, MotorType.kBrushless);
        rightMotor = new SparkMax(ShooterConstants.kRightShooterMotorCANID, MotorType.kBrushless);

        leftMotor.configure(
            ShooterConstants.kLeftMotorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        rightMotor.configure(
            ShooterConstants.kRightMotorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        leftEncoder = leftMotor.getAbsoluteEncoder();
        rightEncoder = rightMotor.getAbsoluteEncoder();

        leftClosedLoopController = leftMotor.getClosedLoopController();
        rightClosedLoopController = rightMotor.getClosedLoopController();

        // TODO Set this to the initial startup speed
        targetSpeeds = null;

        rightRelative = rightMotor.getEncoder();
    }

    @Override
    public void periodic() {
        Logger.recordOutput(
            "Shooter/TargetRPM", 
            targetSpeeds == null ? 0 : targetSpeeds.getSpeedRPM()
        );

        Logger.recordOutput("Shooter/LeftRollers/CurrentRPM", leftEncoder.getVelocity());
        Logger.recordOutput("Shooter/RightRollers/CurrentRPM", rightEncoder.getVelocity());

        Logger.recordOutput("Shooter/RightRollers/rightmotor", rightRelative.getVelocity());


        // TODO How does the SparkMAX controller determine "at setpoint"? Is there any tolerance?
        Logger.recordOutput("Shooter/LeftRollers/AtSetpoint", leftClosedLoopController.isAtSetpoint());
        Logger.recordOutput("Shooter/RightRollers/AtSetpoint", rightClosedLoopController.isAtSetpoint());
    }

    public Command maintainSpeed(ShooterSpeeds speeds) {
        return run(() -> {
            targetSpeeds = speeds;

            if(targetSpeeds == null) {
                leftMotor.disable();
                rightMotor.disable();
            } else {
                leftClosedLoopController.setSetpoint(
                    targetSpeeds.getSpeedRPM(), 
                    ControlType.kVelocity
                );

                rightClosedLoopController.setSetpoint(
                    targetSpeeds.getSpeedRPM(), 
                    ControlType.kVelocity
                );
            }
        });
    }

    public Command manualSpeed(DoubleSupplier speed) {
        return run(() -> {
            targetSpeeds = null;
            
            leftMotor.set(speed.getAsDouble() * ShooterConstants.kMaxManualSpeedMultiplier);
            rightMotor.set(speed.getAsDouble() * ShooterConstants.kMaxManualSpeedMultiplier);
        });
    }

    public Command stop() {
        return manualSpeed(() -> 0);
    }

    public double getAverageActualSpeeds() {
        return (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2;
    }

    public Optional<ShooterSpeeds> getTargetSpeeds() {
        return Optional.ofNullable(targetSpeeds);
    }
}
