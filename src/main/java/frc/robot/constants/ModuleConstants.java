package frc.robot.constants;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;

public class ModuleConstants {
    // DRIVING MOTOR CONFIG (Kraken)
    public static final double kDrivingMotorReduction = (14.0 * 28.0 * 15.0) / (50 * 16 * 45);

    public static final double kDrivingMotorFeedSpeedRPS = KrakenMotorConstants.kFreeSpeedRPM / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDriveWheelFreeSpeedRPS = (kDrivingMotorFeedSpeedRPS * kWheelCircumferenceMeters) /
        kDrivingMotorReduction;
    public static final double kDrivingFactor = kWheelDiameterMeters * Math.PI / kDrivingMotorReduction;
    public static final double kDrivingVelocityFeedForward = 1 / kDriveWheelFreeSpeedRPS;
    
    // TODO Hold over from 2025, adjust?
    public static final double kDriveP = .04;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kDriveS = 0;
    public static final double kDriveV = kDrivingVelocityFeedForward;
    public static final double kDriveA = 0;

    // TODO Hold over from 2025, adjust?
    public static final int kDriveMotorStatorCurrentLimit = 100;
    public static final int kDriveMotorSupplyCurrentLimit = 65;

    // TODO Hold over from 2025, adjust?
    public static final InvertedValue kDriveInversionState = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue kDriveIdleMode = NeutralModeValue.Brake;

    // TURNING MOTOR CONFIG (NEO)
    public static final double kTurningMotorReduction = 150.0/7.0;
    public static final double kTurningFactor = 2 * Math.PI / kTurningMotorReduction;
    // TODO Adjust? Let over from 2025
    public static final double kTurnP = 1;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;

    public static final boolean kIsEncoderInverted = false;

    // TODO How sensitive is too sensitive?
    public static final double kAutoResetPositionDeadband = Units.degreesToRadians(.25);
    
    public static final int kTurnMotorCurrentLimit = 20;

    public static final IdleMode kTurnIdleMode = IdleMode.kBrake;

    // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A CONFIGURATION ITEM

    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    public static final FeedbackConfigs kDriveFeedConfig = new FeedbackConfigs();
    public static final CurrentLimitsConfigs kDriveCurrentLimitConfig = new CurrentLimitsConfigs();
    public static final MotorOutputConfigs kDriveMotorConfig = new MotorOutputConfigs();
    public static final AudioConfigs kAudioConfig = new AudioConfigs();
    public static final Slot0Configs kDriveSlot0Config = new Slot0Configs();

    static {
        kDriveFeedConfig.SensorToMechanismRatio = kDrivingMotorReduction;

        kDriveCurrentLimitConfig.StatorCurrentLimitEnable = true;
        kDriveCurrentLimitConfig.SupplyCurrentLimitEnable = true;
        kDriveCurrentLimitConfig.StatorCurrentLimit = kDriveMotorStatorCurrentLimit;
        kDriveCurrentLimitConfig.SupplyCurrentLimit = kDriveMotorSupplyCurrentLimit;

        kDriveMotorConfig.Inverted = kDriveInversionState;
        kDriveMotorConfig.NeutralMode = kDriveIdleMode;

        kAudioConfig.AllowMusicDurDisable = true;

        kDriveSlot0Config.kP = kDriveP;
        kDriveSlot0Config.kI = kDriveI;
        kDriveSlot0Config.kD = kDriveD;
        kDriveSlot0Config.kS = kDriveS;
        kDriveSlot0Config.kV = kDriveV;
        kDriveSlot0Config.kA = kDriveA;

        turningConfig
            .idleMode(kTurnIdleMode)
            .smartCurrentLimit(kTurnMotorCurrentLimit)
            .inverted(true);
        turningConfig.encoder
            //.inverted(true)
            .positionConversionFactor(kTurningFactor)
            .velocityConversionFactor(kTurningFactor / 60.0);
        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kTurnP, kTurnI, kTurnD)
            .outputRange(-1, 1)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 2 * Math.PI);
            
    }
}
