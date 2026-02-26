package frc.robot.constants;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public enum ShooterSpeeds {
        kHubSpeed(0),
        kFeedSpeed(0);

        private double speedMPS;

        private ShooterSpeeds(double speedMPS) {
            this.speedMPS = speedMPS;
        }

        public double getSpeedMPS() {
            return speedMPS;
        }
    }

    // TODO Conversion factor?

    public static final double kWheelDiameter = Units.inchesToMeters(4);

    // TODO Real values
    public static final int kLeftShooterMotor1CANID = 0;
    public static final int kLeftShooterMotor2CANID = 0;
    public static final int kRightShooterMotor1CANID = 0;
    public static final int kRightShooterMotor2CANID = 0;

    public static final boolean kLeftShooterMotor1Inverted = false;
    public static final boolean kLeftShooterMotor2Inverted = false;
    public static final boolean kRightShooterMotor1Inverted = false;
    public static final boolean kRightShooterMotor2Inverted = false;

    public static final double kLeftP = 0;
    public static final double kLeftI = 0;
    public static final double kLeftD = 0;
    public static final double kLeftS = 0;
    public static final double kLeftV = 0.21;
    public static final double kLeftA = 0;

    public static final double kRightP = 0;
    public static final double kRightI = 0;
    public static final double kRightD = 0;
    public static final double kRightS = 0;
    public static final double kRightV = 0.21;
    public static final double kRightA = 0;

    public static final double kShooterHeightMeters = 0;

    // TODO Is this value sane?
    public static final int kCurrentLimit = 50;

    public static final IdleMode kShooterIdleMode = IdleMode.kCoast;

    // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A CONFIGURATION ITEM

    public static final SparkMaxConfig kLeftMotor1Config = new SparkMaxConfig();
    public static final SparkMaxConfig kLeftMotor2Config = new SparkMaxConfig();
    public static final SparkMaxConfig kRightMotor1Config = new SparkMaxConfig();
    public static final SparkMaxConfig kRightMotor2Config = new SparkMaxConfig();


    static {
        kLeftMotor1Config
            .idleMode(kShooterIdleMode)
            .smartCurrentLimit(kCurrentLimit)
            .inverted(kLeftShooterMotor1Inverted);
        kLeftMotor1Config.absoluteEncoder
            .positionConversionFactor(kWheelDiameter * Math.PI)
            .velocityConversionFactor(kWheelDiameter * Math.PI / 60);
        kLeftMotor1Config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(kLeftP, kLeftI, kLeftD)
            .outputRange(-1, 1)
            .feedForward
                .sva(kLeftS, kLeftV, kLeftA);

        kLeftMotor2Config
            .idleMode(kShooterIdleMode)
            .smartCurrentLimit(kCurrentLimit)
            .inverted(kLeftShooterMotor2Inverted)
            .follow(kLeftShooterMotor1CANID);

        kRightMotor1Config
            .idleMode(kShooterIdleMode)
            .smartCurrentLimit(kCurrentLimit)
            .inverted(kRightShooterMotor1Inverted);
        kRightMotor1Config.absoluteEncoder
            .positionConversionFactor(kWheelDiameter * Math.PI)
            .velocityConversionFactor(kWheelDiameter * Math.PI / 60);
        kRightMotor1Config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(kRightP, kRightI, kRightD)
            .outputRange(-1, 1)
            .feedForward
                .sva(kRightS, kRightV, kRightA);

        kRightMotor2Config
            .idleMode(kShooterIdleMode)
            .smartCurrentLimit(kCurrentLimit)
            .inverted(kRightShooterMotor2Inverted)
            .follow(kRightShooterMotor1CANID);
    }
}
