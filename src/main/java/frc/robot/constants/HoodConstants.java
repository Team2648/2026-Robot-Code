package frc.robot.constants;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Map;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.ShooterConstants.ShooterSpeeds;

public class HoodConstants {
    // TODO Real Values
    public static final int kMotorCANID = 12;

    public static final double kConversionFactor = (1.0/3.0)*(8.0/147.0)*2*Math.PI;

    public static final double kP = 1.75;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0.435;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kStartupAngle = 0.0;
    public static final double kMaxManualSpeedMultiplier = 0.1;
    public static final double kTolerance = Math.toRadians(0.5);

    public static final double kAmpsToTriggerPositionReset = 10;

    // TODO This is just barely longer than the default frame time for output current information
    // Should this be longer?
    public static final double kTimeAboveThresholdToReset = .25;

    public static final int kCurrentLimit = 15;

    public static final boolean kInverted = true;
    public static final boolean kUseInterpolatorForAngle = false;

    public static final IdleMode kIdleMode = IdleMode.kBrake;

    public static final Map<ShooterSpeeds, InterpolatingDoubleTreeMap> kHoodInterpolators = Map.of(
        ShooterSpeeds.kHubSpeed, new InterpolatingDoubleTreeMap()
    );
    // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A CONFIGURATION ITEM

    public static final SparkMaxConfig kConfig = new SparkMaxConfig();

    static {
        kConfig
            .idleMode(kIdleMode)
            .inverted(kInverted)
            .smartCurrentLimit(kCurrentLimit);
        kConfig.encoder
            .positionConversionFactor(kConversionFactor)
            .velocityConversionFactor(kConversionFactor / 60);
        kConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .outputRange(-1, 1)
            .allowedClosedLoopError(kTolerance, ClosedLoopSlot.kSlot0)
            .feedForward
                .sva(kS, kV, kA);

        kHoodInterpolators.get(ShooterSpeeds.kHubSpeed).put(
            Double.valueOf(Units.inchesToMeters(22.2 + 40)), 
            Double.valueOf(Units.degreesToRadians(10)));

        kHoodInterpolators.get(ShooterSpeeds.kHubSpeed).put(
            Double.valueOf(Units.inchesToMeters(22.2 + 60)), 
            Double.valueOf(Units.degreesToRadians(13)));

        kHoodInterpolators.get(ShooterSpeeds.kHubSpeed).put(
            Double.valueOf(Units.inchesToMeters(22.2 + 80)),
            Double.valueOf(Units.degreesToRadians(17)));

        kHoodInterpolators.get(ShooterSpeeds.kHubSpeed).put(
            Double.valueOf(Units.inchesToMeters(22.2 + 100)), 
            Double.valueOf(Units.degreesToRadians(21)));

        kHoodInterpolators.get(ShooterSpeeds.kHubSpeed).put(
            Double.valueOf(Units.inchesToMeters(22.2 + 120)), 
            Double.valueOf(Units.degreesToRadians(24)));
    }
}
