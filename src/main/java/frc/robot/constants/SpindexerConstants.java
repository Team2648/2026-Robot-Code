package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class SpindexerConstants {
    // TODO Real values
    public static final int kMotorCANID = 0;

    public static final int kStatorCurrentLimit = 80;
    public static final int kSupplyCurrentLimit = 30;

    public static final InvertedValue kInversionState = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue kIdleMode = NeutralModeValue.Brake;

    // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A CONFIGURATION ITEM

    public static final CurrentLimitsConfigs kCurrentLimitConfig = new CurrentLimitsConfigs();
    public static final MotorOutputConfigs kMotorConfig = new MotorOutputConfigs();
    
    static {
        kCurrentLimitConfig.StatorCurrentLimitEnable = true;
        kCurrentLimitConfig.SupplyCurrentLimitEnable = true;
        kCurrentLimitConfig.StatorCurrentLimit = kStatorCurrentLimit;
        kCurrentLimitConfig.SupplyCurrentLimit = kSupplyCurrentLimit;

        kMotorConfig.Inverted = kInversionState;
        kMotorConfig.NeutralMode = kIdleMode;
    }
}
