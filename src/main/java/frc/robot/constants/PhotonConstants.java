package frc.robot.constants;

import java.util.List;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.utilities.PhotonVisionConfig;

public class PhotonConstants {
    public static final String kCamera1Name = "pv1";
    public static final String kCamera2Name = "pv2";

    // TODO Need actual values for all of this
    public static final Transform3d kCamera1RobotToCam = new Transform3d();
    public static final Transform3d kCamera2RobotToCam = new Transform3d();

    public static final double kCamera1HeightMeters = 0;
    public static final double kCamera1PitchRadians = 0;
    public static final double kCamera2HeightMeters = 0;
    public static final double kCamera2PitchRadians = 0;

    // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A CONFIGURATION ITEM

    public static final List<PhotonVisionConfig> configs = List.of(
        new PhotonVisionConfig(kCamera1Name, kCamera1RobotToCam, kCamera1HeightMeters, kCamera1PitchRadians),
        new PhotonVisionConfig(kCamera2Name, kCamera2RobotToCam, kCamera2HeightMeters, kCamera2PitchRadians)
    );
}