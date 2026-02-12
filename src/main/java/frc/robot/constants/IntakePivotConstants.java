package frc.robot.constants;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakePivotConstants {
    // TODO Real values
    public enum IntakePivotPosition {
        kUp(0),
        kDown(0);
        private double positionRadians;

        private IntakePivotPosition(double positionRadians) {
            this.positionRadians = positionRadians;
        }

        public double getPositionRadians() {
            return positionRadians;
        }
    }

    public static final int kLeftMotorCANID = 0;
    public static final int kRightMotorCANID = 0;

    public static final double kConversionFactor = 0;
    public static final double kMaxManualSpeedMultiplier = .5;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final boolean kInvertMotors = false;

    public static final int kCurrentLimit = 30;

    public static final IdleMode kIdleMode = IdleMode.kBrake;

    // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A CONFIGURATION ITEM
    
    public static final SparkMaxConfig KLeftMotorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kRightMotorConfig = new SparkMaxConfig();

    static {
        KLeftMotorConfig
            .idleMode(kIdleMode)
            .smartCurrentLimit(kCurrentLimit)
            .inverted(kInvertMotors);
        KLeftMotorConfig.absoluteEncoder
            .positionConversionFactor(kConversionFactor)
            .velocityConversionFactor(kConversionFactor / 60);
        KLeftMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(kP, kI, kD)
            .outputRange(-1, 1)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 2 * Math.PI)
            .feedForward
                .sva(kS, kV, kA);

        kRightMotorConfig
            .idleMode(kIdleMode)
            .smartCurrentLimit(kCurrentLimit)
            .inverted(kInvertMotors)
            .follow(kLeftMotorCANID);
    }
}
