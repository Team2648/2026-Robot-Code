package frc.robot.utilities;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.constants.ModuleConstants;

/*
 * This thread
 * 
 * https://www.chiefdelphi.com/t/best-easiest-way-to-connect-wire-and-program-thrifty-absolute-magnetic-encoder-to-rev-spark-max-motor-controller/439040/30
 * 
 * implies that the best use of the thrifty absolute encoder is to use it as a reference for the Spark relative encoder and then
 * used the closed loop control on the controller for turning
 * 
 * IDK if that's really necessary, the read rate of the analog ports is 100HZ, I suppose the only benefit is the higher rate of
 * the controller closed loop controller. 
 */
public class SwerveModule {
    private TalonFX drive;
    private SparkMax turning;

    private RelativeEncoder turningRelativeEncoder;

    private AnalogEncoder turningAbsoluteEncoder;

    private SparkClosedLoopController turningClosedLoopController;

    private VelocityVoltage driveVelocityRequest;

    private String moduleName;

    private SwerveModuleState lastTargetState;
    private SwerveModuleState lastTargetStateOptimized;

    private boolean isAbsoluteEncoderDisabled;
    private boolean turningEncoderAutoRezeroEnabled;

    /**
     * Builds the swerve module but with the Absolute Encoder disabled.
     * 
     * This constructor assumes you zeroed the swerve modules (faced all the bevel gears to the left)
     * before booting up the robot.
     * 
     * @param moduleName The module name, Front Left, Front Right, etc.
     * @param drivingCANID The CAN ID of the Kraken used to drive the module wheel
     * @param turningCANID The CAN ID of the Spark MAX used to turn the module wheel
     */
    public SwerveModule(String moduleName, int drivingCANID, int turningCANID) {
        this(moduleName, drivingCANID, turningCANID, -1, -1);
    }

    /**
     * Builds the swerve module with the normal features, disables automatic rezeroing of the turning encoder
     * from the absolute encoder. 
     * 
     * @param moduleName The module name, Front Left, Front Right, etc.
     * @param drivingCANID The CAN ID of the Kraken used to drive the module wheel
     * @param turningCANID The CAN ID of the Spark MAX used to turn the module wheel
     * @param analogEncoderID The Analog In port ID for the Thrify Absolute Encoder
     * @param analogEncoderOffset The angular offset for the absolute encoder to achieve 0 position on the module 
     */
    public SwerveModule(String moduleName, int drivingCANID, int turningCANID, int analogEncoderID, double analogEncoderOffset) {
        this(moduleName, drivingCANID, turningCANID, analogEncoderID, analogEncoderOffset, false);
    }

    /**
     * Builds the swerve module with the normal features, and gives the option to enable automatic turning encoder rezeroing
     * when the turning motor is not moving
     * 
     * @param moduleName The module name, Front Left, Front Right, etc.
     * @param drivingCANID The CAN ID of the Kraken used to drive the module wheel
     * @param turningCANID The CAN ID of the Spark MAX used to turn the module wheel
     * @param analogEncoderID The Analog In port ID for the Thrify Absolute Encoder
     * @param analogEncoderOffset The angular offset for the absolute encoder to achieve 0 position on the module 
     * @param turningEncoderAutoRezeroEnabled Should the turning encoder in the NEO automatically rezero from the absolute encoder
     */
    public SwerveModule(String moduleName, int drivingCANID, int turningCANID, 
        int analogEncoderID, double analogEncoderOffset, boolean turningEncoderAutoRezeroEnabled) {
        isAbsoluteEncoderDisabled = (analogEncoderID == -1) || (analogEncoderOffset < 0);

        drive = new TalonFX(drivingCANID);
        turning = new SparkMax(turningCANID, MotorType.kBrushless);

        turningRelativeEncoder = turning.getEncoder();

        if(!isAbsoluteEncoderDisabled) {
            turningAbsoluteEncoder = new AnalogEncoder(analogEncoderID, 2 * Math.PI, analogEncoderOffset);
        }
        
        turningClosedLoopController = turning.getClosedLoopController();

        drive.getConfigurator().apply(ModuleConstants.kDriveCurrentLimitConfig);
        drive.getConfigurator().apply(ModuleConstants.kDriveFeedConfig);
        drive.getConfigurator().apply(ModuleConstants.kDriveMotorConfig);
        drive.getConfigurator().apply(ModuleConstants.kAudioConfig);
        drive.getConfigurator().apply(ModuleConstants.kDriveSlot0Config);

        turning.configure(
            ModuleConstants.turningConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        driveVelocityRequest = new VelocityVoltage(0);

        if(isAbsoluteEncoderDisabled){
            turningRelativeEncoder.setPosition(0);
        } else {
            turningRelativeEncoder.setPosition(turningAbsoluteEncoder.get());
        }
        
        drive.setPosition(0);

        this.lastTargetState = getState();
        this.lastTargetStateOptimized = getState();

        this.turningEncoderAutoRezeroEnabled = turningEncoderAutoRezeroEnabled;

        this.moduleName = "Drivetrain/Modules/" + moduleName;
    }

    public void periodic() {
        
        if(!isAbsoluteEncoderDisabled) {
            Logger.recordOutput(moduleName + "/AbsoluteEncoder/Position", turningAbsoluteEncoder.get());
        }
        
        Logger.recordOutput(moduleName + "/ModuleTargetState", lastTargetState);
        Logger.recordOutput(moduleName + "/ModuleTargetStateOptimized", lastTargetStateOptimized);
        Logger.recordOutput(moduleName + "/SwerveModuleState", getState());
        Logger.recordOutput(moduleName + "/SwerveModulePosition", getPosition());
        Logger.recordOutput(moduleName + "/RelativeEncoderPosition", getTurningEncoderPosition());

        // TODO Re-enable this? Was turned off when there was drivetrain issues
        // Now that there aren't, do we try this again?
        /*
        if(!isAbsoluteEncoderDisabled && turningEncoderAutoRezeroEnabled) {
            if(Math.abs(getState().angle.getRadians() - lastTargetState.angle.getRadians()) <= ModuleConstants.kAutoResetPositionDeadband) {
                resetEncoders();
            }
        }*/
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            drive.getVelocity().getValueAsDouble() * ModuleConstants.kWheelCircumferenceMeters, 
            new Rotation2d(getTurningEncoderPosition())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            drive.getPosition().getValueAsDouble() * ModuleConstants.kWheelCircumferenceMeters, 
            new Rotation2d(getTurningEncoderPosition())
        );
    }

    public void disableOutput() {
        drive.disable();
        turning.disable();
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        lastTargetState = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        // TODO is this really necessary, the offset is managed by the Absolute Encoder
        // and its "source of truth" behavior in relation to the relative encoder
        // Probably doesn't *hurt* that it's here, but it may not be needed
        desiredState.optimize(new Rotation2d(getTurningEncoderPosition()));

        lastTargetStateOptimized = desiredState;

        drive.setControl(
            driveVelocityRequest.withVelocity(
                desiredState.speedMetersPerSecond / ModuleConstants.kWheelCircumferenceMeters
            ).withFeedForward(
                desiredState.speedMetersPerSecond / ModuleConstants.kWheelCircumferenceMeters
            )
        );

        turningClosedLoopController.setSetpoint(
            desiredState.angle.getRadians(), 
            ControlType.kPosition
        );
    }

    public double getTurningEncoderPosition() {
        return turningRelativeEncoder.getPosition() * (ModuleConstants.kIsEncoderInverted ? -1 : 1);
    }

    public void resetEncoders() {
        drive.setPosition(0);

        zeroTurningEncoder();
    }

    public void zeroTurningEncoder() {
        if(isAbsoluteEncoderDisabled) {
            turningRelativeEncoder.setPosition(0);
        } else {
            turningRelativeEncoder.setPosition(turningAbsoluteEncoder.get());
        }
    }
}
