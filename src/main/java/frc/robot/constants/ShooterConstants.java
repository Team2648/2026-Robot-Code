package frc.robot.constants;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public enum ShooterSpeeds {
        kHubSpeed(3000.0),
        kFeedSpeed(3000.0);

        private double speedMPS;
        private double speedRPM;

        private ShooterSpeeds(double speedRPM) {
            this.speedMPS = speedRPM * kWheelDiameter*Math.PI;
            this.speedRPM = speedRPM;
        }

        public double getSpeedMPS() {
            return speedMPS * kWheelDiameter*Math.PI;
        }

        public double getSpeedRPM(){
            return speedRPM;
        }
    }

    // TODO Conversion factor?

    public static final double kWheelDiameter = Units.inchesToMeters(4);

    // TODO Real values
    public static final int kLeftShooterMotorCANID = 2;
    public static final int kRightShooterMotorCANID = 5;

    public static final boolean kLeftShooterMotorInverted = true;
    public static final boolean kRightShooterMotorInverted = false;

    public static final double kLeftP = 0.0;//0.001;
    public static final double kLeftI = 0;
    public static final double kLeftD = 0;
    public static final double kLeftS = 0;
    public static final double kLeftV = 0.00121;
    public static final double kLeftA = 0;

    public static final double kRightP = 0.001;//0.1;
    public static final double kRightI = 0;
    public static final double kRightD = 0.000;
    public static final double kRightS = 0;
    public static final double kRightV = 0.00121;
    public static final double kRightA = 0;

    public static final double kMaxManualSpeedMultiplier = 1;

    public static final double kShooterHeightMeters = 0;

    // TODO Is this value sane?
    public static final int kCurrentLimit = 60;

    public static final IdleMode kShooterIdleMode = IdleMode.kCoast;

    // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A CONFIGURATION ITEM

    public static final SparkMaxConfig kLeftMotorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kRightMotorConfig = new SparkMaxConfig();


    static {
        kLeftMotorConfig
            .idleMode(kShooterIdleMode)
            .smartCurrentLimit(kCurrentLimit)
            .inverted(kLeftShooterMotorInverted);
        kLeftMotorConfig.absoluteEncoder
            .positionConversionFactor(1)
            .velocityConversionFactor(60);
        kLeftMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(kLeftP, kLeftI, kLeftD)
            .outputRange(-1, 1)
            .feedForward
                .sva(kLeftS, kLeftV, kLeftA);

        kRightMotorConfig
            .idleMode(kShooterIdleMode)
            .smartCurrentLimit(kCurrentLimit)
            .inverted(kRightShooterMotorInverted);
        kRightMotorConfig.absoluteEncoder
            .positionConversionFactor(1)
            .velocityConversionFactor(60)
            .inverted(true);
        kRightMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(kRightP, kRightI, kRightD)
            .outputRange(-1, 1)
            .feedForward
                .sva(kRightS, kRightV, kRightA);
    }
}
