package frc.robot.constants;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public enum ShooterSpeeds {
        kHubSpeed(0, 0),
        kFeedSpeed(0, 0);

        private double frontRollerMPS;
        private double rearRollerMPS;

        private ShooterSpeeds(double frontRollerMPS, double rearRollerMPS) {
            this.frontRollerMPS = frontRollerMPS;
            this.rearRollerMPS = rearRollerMPS;
        }

        public double getFrontRollerMPS() {
            return frontRollerMPS;
        }

        public double getRearRollerMPS() {
            return rearRollerMPS;
        }
    }

    public static final double kWheelDiameter = Units.inchesToMeters(6);

    // TODO Real values
    public static final int kFrontShooterMotor1CANID = 0;
    public static final int kFrontShooterMotor2CANID = 0;
    public static final int kRearShooterMotor1CANID = 0;
    public static final int kRearShooterMotor2CANID = 0;

    public static final boolean kFrontShooterMotor1Inverted = false;
    public static final boolean kFrontShooterMotor2Inverted = false;
    public static final boolean kRearShooterMotor1Inverted = false;
    public static final boolean kRearShooterMotor2Inverted = false;

    public static final double kFrontP = 0;
    public static final double kFrontI = 0;
    public static final double kFrontD = 0;
    public static final double kFrontS = 0;
    public static final double kFrontV = 0;
    public static final double kFrontA = 0;

    public static final double kRearP = 0;
    public static final double kRearI = 0;
    public static final double kRearD = 0;
    public static final double kRearS = 0;
    public static final double kRearV = 0;
    public static final double kRearA = 0;

    // TODO Is this value sane?
    public static final int kCurrentLimit = 30;

    public static final IdleMode kShooterIdleMode = IdleMode.kCoast;

    // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A CONFIGURATION ITEM

    public static final SparkMaxConfig kFrontMotor1Config = new SparkMaxConfig();
    public static final SparkMaxConfig kFrontMotor2Config = new SparkMaxConfig();
    public static final SparkMaxConfig kRearMotor1Config = new SparkMaxConfig();
    public static final SparkMaxConfig kRearMotor2Config = new SparkMaxConfig();


    static {
        kFrontMotor1Config
            .idleMode(kShooterIdleMode)
            .smartCurrentLimit(kCurrentLimit)
            .inverted(kFrontShooterMotor1Inverted);
        kFrontMotor1Config.absoluteEncoder
            .positionConversionFactor(kWheelDiameter * Math.PI)
            .velocityConversionFactor(kWheelDiameter * Math.PI / 60);
        kFrontMotor1Config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(kFrontP, kFrontI, kFrontD)
            .outputRange(-1, 1)
            .feedForward
                .sva(kFrontS, kFrontV, kFrontA);

        kFrontMotor2Config
            .idleMode(kShooterIdleMode)
            .smartCurrentLimit(kCurrentLimit)
            .inverted(kFrontShooterMotor2Inverted)
            .follow(kFrontShooterMotor1CANID);

        kRearMotor1Config
            .idleMode(kShooterIdleMode)
            .smartCurrentLimit(kCurrentLimit)
            .inverted(kRearShooterMotor1Inverted);
        kRearMotor1Config.absoluteEncoder
            .positionConversionFactor(kWheelDiameter * Math.PI)
            .velocityConversionFactor(kWheelDiameter * Math.PI / 60);
        kRearMotor1Config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(kRearP, kRearI, kRearD)
            .outputRange(-1, 1)
            .feedForward
                .sva(kRearS, kRearV, kRearA);

        kRearMotor2Config
            .idleMode(kShooterIdleMode)
            .smartCurrentLimit(kCurrentLimit)
            .inverted(kRearShooterMotor2Inverted)
            .follow(kRearShooterMotor1CANID);
    }
}
