package frc.robot.constants;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ClimberConstants {
    // TODO Real values
    public enum ClimberPositions {
        kStow(0),
        kClimbOffGround(0),
        kUp(0);

        private double positionMeters;

        private ClimberPositions(double positionMeters) {
            this.positionMeters = positionMeters;
        }

        public double getPositionMeters() {
            return positionMeters;
        }
    }

    public static final int kMotorCANID = 0;

    public static final double kConversionFactor = 0;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final boolean kMotorInverted = false;

    public static final int kCurrentLimit = 40;

    public static final IdleMode kIdleMode = IdleMode.kBrake;

    // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A CONFIGURATION ITEM

    public static final SparkMaxConfig kMotorConfig = new SparkMaxConfig();

    static {
        kMotorConfig
            .inverted(kMotorInverted)
            .smartCurrentLimit(kCurrentLimit)
            .idleMode(kIdleMode);
        kMotorConfig.encoder
            .positionConversionFactor(kConversionFactor)
            .velocityConversionFactor(kConversionFactor / 60);
        kMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .outputRange(-1, 1)
            .feedForward
                .sva(kS, kV, kA);
    }
}
