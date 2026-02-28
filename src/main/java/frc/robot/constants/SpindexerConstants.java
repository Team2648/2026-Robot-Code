package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SpindexerConstants {
    // TODO Real values
    public static final int kSpindexerMotorCANID = 0;
    public static final int kFeederMotorCANID = 4;

    public static final int kSpindexerStatorCurrentLimit = 110;
    public static final int kSpindexerSupplyCurrentLimit = 60;
    public static final int kFeederCurrentLimit = 40;

    public static final double kSpindexerSpeed = 1;
    public static final double kFeederSpeed = 1;

    public static final boolean kFeederMotorInverted = true;

    public static final InvertedValue kSpindexerInversionState = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue kSpindexerIdleMode = NeutralModeValue.Coast;

    public static final IdleMode kFeederIdleMode = IdleMode.kBrake;

    // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A CONFIGURATION ITEM

    public static final SparkMaxConfig kFeederConfig = new SparkMaxConfig();

    public static final CurrentLimitsConfigs kSpindexerCurrentLimitConfig = new CurrentLimitsConfigs();
    public static final MotorOutputConfigs kSpindexerMotorConfig = new MotorOutputConfigs();
    
    static {
        kSpindexerCurrentLimitConfig.StatorCurrentLimitEnable = true;
        kSpindexerCurrentLimitConfig.SupplyCurrentLimitEnable = true;
        kSpindexerCurrentLimitConfig.StatorCurrentLimit = kSpindexerStatorCurrentLimit;
        kSpindexerCurrentLimitConfig.SupplyCurrentLimit = kSpindexerSupplyCurrentLimit;

        kSpindexerMotorConfig.Inverted = kSpindexerInversionState;
        kSpindexerMotorConfig.NeutralMode = kSpindexerIdleMode;

        kFeederConfig
            .inverted(kFeederMotorInverted)
            .smartCurrentLimit(kFeederCurrentLimit)
            .idleMode(kFeederIdleMode);
    }
}
