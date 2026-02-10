package frc.robot.constants;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class HoodConstants {
    // TODO Real Values
    public static final int kMotorCANID = 0;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kStartupAngle = 0;

    public static final int kCurrentLimit = 15;

    public static final boolean kInverted = false;

    public static final IdleMode kIdleMode = IdleMode.kBrake;

    // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A CONFIGURATION ITEM

    public static final SparkMaxConfig kConfig = new SparkMaxConfig();

    static {
        kConfig
            .idleMode(kIdleMode)
            .inverted(kInverted)
            .smartCurrentLimit(kCurrentLimit);
        kConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(kP, kI, kD)
            .outputRange(-1, 1)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, Math.PI * 2)
            .feedForward
                .sva(kS, kV, kA);
    }
}
