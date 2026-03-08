package frc.robot.constants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeRollerConstants {
    // TODO Real values
    public static final int kMotorCANID = 20;

    public static final int kCurrentLimit = 65;

    public static final boolean kInvertMotors = true;

    public static final double kSpeed = 1;

    public static final IdleMode kIdleMode = IdleMode.kCoast;

    // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A CONFIGURATION ITEM

    public static final SparkMaxConfig leftMotorConfig = new SparkMaxConfig();

    static {
        leftMotorConfig
            .idleMode(kIdleMode)
            .smartCurrentLimit(kCurrentLimit)
            .inverted(kInvertMotors);
    }
}
