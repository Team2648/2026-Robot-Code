package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;

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
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.ShooterSpeeds;

public class Shooter extends SubsystemBase {
    private SparkMax leftMotor1;
    private SparkMax leftMotor2;
    private SparkMax rightMotor1;
    private SparkMax rightMotor2;

    private AbsoluteEncoder leftEncoder;
    private AbsoluteEncoder rightEncoder;

    private SparkClosedLoopController leftClosedLoopController;
    private SparkClosedLoopController rightClosedLoopController;

    private ShooterSpeeds targetSpeeds;

    public Shooter() {
        leftMotor1 = new SparkMax(ShooterConstants.kLeftShooterMotor1CANID, MotorType.kBrushless);
        leftMotor2 = new SparkMax(ShooterConstants.kLeftShooterMotor2CANID, MotorType.kBrushless);
        rightMotor1 = new SparkMax(ShooterConstants.kRightShooterMotor1CANID, MotorType.kBrushless);
        rightMotor2 = new SparkMax(ShooterConstants.kRightShooterMotor2CANID, MotorType.kBrushless);

        leftMotor1.configure(
            ShooterConstants.kLeftMotor1Config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        rightMotor1.configure(
            ShooterConstants.kRightMotor1Config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        leftMotor2.configure(
            ShooterConstants.kLeftMotor2Config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        rightMotor2.configure(
            ShooterConstants.kRightMotor2Config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        leftEncoder = leftMotor1.getAbsoluteEncoder();
        rightEncoder = rightMotor1.getAbsoluteEncoder();

        leftClosedLoopController = leftMotor1.getClosedLoopController();
        rightClosedLoopController = rightMotor1.getClosedLoopController();

        // TODO Set this to the initial startup speed
        targetSpeeds = null;
    }

    @Override
    public void periodic() {
        Logger.recordOutput(
            "Shooter/TargetMPS", 
            targetSpeeds == null ? 0 : targetSpeeds.getSpeedMPS()
        );

        Logger.recordOutput("Shooter/LeftRollers/CurrentMPS", leftEncoder.getVelocity());
        Logger.recordOutput("Shooter/RightRollers/CurrentMPS", rightEncoder.getVelocity());

        // TODO How does the SparkMAX controller determine "at setpoint"? Is there any tolerance?
        Logger.recordOutput("Shooter/LeftRollers/AtSetpoint", leftClosedLoopController.isAtSetpoint());
        Logger.recordOutput("Shooter/RightRollers/AtSetpoint", rightClosedLoopController.isAtSetpoint());
    }

    public Command maintainSpeed(ShooterSpeeds speeds) {
        targetSpeeds = speeds;

        return run(() -> {
            if(targetSpeeds == null) {
                leftMotor1.disable();
                rightMotor1.disable();
            } else {
                leftClosedLoopController.setSetpoint(
                    targetSpeeds.getSpeedMPS(), 
                    ControlType.kVelocity
                );

                rightClosedLoopController.setSetpoint(
                    targetSpeeds.getSpeedMPS(), 
                    ControlType.kVelocity
                );
            }
        });
    }

    public Command manualSpeed(DoubleSupplier speed) {
        targetSpeeds = null;

        return run(() -> {
            leftMotor1.set(speed.getAsDouble());
            rightMotor1.set(speed.getAsDouble());
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
