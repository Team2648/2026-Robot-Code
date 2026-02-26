package frc.robot.constants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeRollerConstants {
    // TODO Real values
    public static final int kLeftMotorCANID = 0;
    public static final int kRightMotorCANID = 0;

    public static final int kCurrentLimit = 40;

    public static final boolean kInvertMotors = false;

    public static final IdleMode kIdleMode = IdleMode.kCoast;

    // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A CONFIGURATION ITEM

    public static final SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

    static {
        leftMotorConfig
            .idleMode(kIdleMode)
            .smartCurrentLimit(kCurrentLimit)
            .inverted(kInvertMotors);

        rightMotorConfig
            .idleMode(kIdleMode)
            .smartCurrentLimit(kCurrentLimit)
            .inverted(kInvertMotors)
            .follow(kLeftMotorCANID);
    }
}
