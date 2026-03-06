package frc.robot.constants;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakePivotConstants {
    // TODO Real values
    public enum IntakePivotPosition {
        kUp(Math.toRadians(116.0)),
        kDown(Math.toRadians(0.0));
        private double positionRadians;

        private IntakePivotPosition(double positionRadians) {
            this.positionRadians = positionRadians;
        }

        public double getPositionRadians() {
            return positionRadians;
        }
    }

    public static final int kLeftMotorCANID = 16;
    public static final int kRightMotorCANID = 9;

    public static final double kConversionFactor = 60.0/11.0*60.0/18.0*38.0/16.0;

    // Ultra conservative multiplier to prevent 1/8" lexan destruction, modify at your own peril
    public static final double kMaxManualSpeedMultiplier = .3;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 5.26;
    public static final double kA = 0.05;
    public static final double kG = 0.25;

    public static final boolean kInvertMotors = false;

    public static final int kCurrentLimit = 30;

    public static final IdleMode kIdleMode = IdleMode.kCoast;

    // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A CONFIGURATION ITEM
    
    public static final SparkMaxConfig KLeftMotorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kRightMotorConfig = new SparkMaxConfig();

    static {
        KLeftMotorConfig
            .idleMode(kIdleMode)
            .smartCurrentLimit(kCurrentLimit)
            .inverted(false);
        KLeftMotorConfig.encoder
            .positionConversionFactor(kConversionFactor)
            .velocityConversionFactor(kConversionFactor / 60);
        KLeftMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .outputRange(-1, 1)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 2 * Math.PI)
            .feedForward
                .svag(kS, kV, kA, kG);

        kRightMotorConfig
            .idleMode(kIdleMode)
            .smartCurrentLimit(kCurrentLimit)
            .inverted(true)
            ;//.follow(kLeftMotorCANID);
    }
}
