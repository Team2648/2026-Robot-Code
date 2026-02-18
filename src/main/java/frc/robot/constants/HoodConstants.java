package frc.robot.constants;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Filesystem;

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
    public static final double kMaxManualSpeedMultiplier = .5;

    public static final int kCurrentLimit = 15;

    public static final boolean kInverted = false;
    public static final boolean kUseInterpolatorForAngle = false;

    public static final IdleMode kIdleMode = IdleMode.kBrake;

    // TODO This needs to be filled in from some source
    public static final InterpolatingDoubleTreeMap kDistanceToAngle = new InterpolatingDoubleTreeMap();

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


        File interpolatorFile = Path.of(
            Filesystem.getDeployDirectory().getAbsolutePath().toString(), 
            "interpolatorData.csv"
        ).toFile();

        if(interpolatorFile.exists()) {
            try (BufferedReader reader = new BufferedReader(new FileReader(interpolatorFile))) {
                reader.lines().forEach((s) -> {
                    if(s.trim() != "") { //Empty or whitespace line protection
                        String[] lineSplit = s.split(",");

                        kDistanceToAngle.put(
                            Double.valueOf(lineSplit[0].replace("\"", "")), 
                            Double.valueOf(lineSplit[1].replace("\"", ""))
                        );
                    }
                });
            } catch (IOException e) {
                // This condition is never reached because of the if exists line above
            }
        }
    }
}
