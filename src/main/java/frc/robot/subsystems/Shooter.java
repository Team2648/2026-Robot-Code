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
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.ShooterSpeeds;

public class Shooter extends SubsystemBase {
    private SparkMax frontMotor1;
    private SparkMax frontMotor2;
    private SparkMax rearMotor1;
    private SparkMax rearMotor2;

    private AbsoluteEncoder frontEncoder;
    private AbsoluteEncoder rearEncoder;

    private SparkClosedLoopController frontClosedLoopController;
    private SparkClosedLoopController rearClosedLoopController;

    private ShooterSpeeds targetSpeeds;

    public Shooter() {
        frontMotor1 = new SparkMax(ShooterConstants.kFrontShooterMotor1CANID, MotorType.kBrushless);
        frontMotor2 = new SparkMax(ShooterConstants.kFrontShooterMotor2CANID, MotorType.kBrushless);
        rearMotor1 = new SparkMax(ShooterConstants.kRearShooterMotor1CANID, MotorType.kBrushless);
        rearMotor2 = new SparkMax(ShooterConstants.kRearShooterMotor2CANID, MotorType.kBrushless);

        frontMotor1.configure(
            ShooterConstants.kFrontMotor1Config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        rearMotor1.configure(
            ShooterConstants.kRearMotor1Config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        frontMotor2.configure(
            ShooterConstants.kFrontMotor2Config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        rearMotor2.configure(
            ShooterConstants.kRearMotor2Config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        frontEncoder = frontMotor1.getAbsoluteEncoder();
        rearEncoder = rearMotor1.getAbsoluteEncoder();

        frontClosedLoopController = frontMotor1.getClosedLoopController();
        rearClosedLoopController = rearMotor1.getClosedLoopController();

        // TODO Set this to the initial startup speed
        targetSpeeds = null;
    }

    @Override
    public void periodic() {
        Logger.recordOutput(
            "Shooter/FrontRollers/TargetMPS", 
            targetSpeeds == null ? 0 : targetSpeeds.getFrontRollerMPS()
        );

        Logger.recordOutput(
            "Shooter/RearRollers/TargetMPS", 
            targetSpeeds == null ? 0 : targetSpeeds.getRearRollerMPS()
        );

        Logger.recordOutput("Shooter/FrontRollers/CurrentMPS", frontEncoder.getVelocity());
        Logger.recordOutput("Shooter/RearRollers/CurrentMPS", rearEncoder.getVelocity());

        // TODO How does the SparkMAX controller determine "at setpoint"? Is there any tolerance?
        Logger.recordOutput("Shooter/FrontRollers/AtSetpoint", frontClosedLoopController.isAtSetpoint());
        Logger.recordOutput("Shooter/RearRollers/AtSetpoint", rearClosedLoopController.isAtSetpoint());
    }

    public Command maintainSpeed(ShooterSpeeds speeds) {
        targetSpeeds = speeds;

        return run(() -> {
            if(targetSpeeds == null) {
                frontMotor1.disable();
                rearMotor1.disable();
            } else {
                frontClosedLoopController.setSetpoint(
                    targetSpeeds.getFrontRollerMPS(), 
                    ControlType.kVelocity
                );

                rearClosedLoopController.setSetpoint(
                    targetSpeeds.getRearRollerMPS(), 
                    ControlType.kVelocity
                );
            }
        });
    }

    public Command stop() {
        return maintainSpeed(null);
    }

    public Optional<ShooterSpeeds> getTargetSpeeds() {
        return Optional.ofNullable(targetSpeeds);
    }
}
