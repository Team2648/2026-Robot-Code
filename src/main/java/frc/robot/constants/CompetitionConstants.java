package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class CompetitionConstants {
    // THIS SHOULD BE FALSE DURING COMPETITION PLAY
    public static final boolean kLogToNetworkTables = true;

    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(
        AprilTagFields.kDefaultField
    );

    public static final Pose2d kBlueHubLocation = new Pose2d(
        Units.inchesToMeters(182.11), 
        Units.inchesToMeters(158.84), 
        Rotation2d.fromDegrees(0)
    );

    public static final Pose2d kRedHubLocation = new Pose2d(
        Units.inchesToMeters(182.11 + 143.5 * 2),
        Units.inchesToMeters(158.84),
        Rotation2d.fromDegrees(0)
    );

}
