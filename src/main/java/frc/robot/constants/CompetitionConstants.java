package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class CompetitionConstants {
    // THIS SHOULD BE FALSE DURING COMPETITION PLAY
    public static final boolean kLogToNetworkTables = true;

    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(
        AprilTagFields.kDefaultField
    );

    public static final double kHubGoalHeightMeters = Units.inchesToMeters(72);

    // TODO Real Values
    public static final Transform3d kRobotToShooter = new Transform3d();

    public static final Pose2d kBlueHubLocation = new Pose2d(
        Units.inchesToMeters(182.11), 
        Units.inchesToMeters(158.84), 
        Rotation2d.fromDegrees(0)
    );

    // TODO The origination value produced by the April Tag Field Layout object may
    // influence what this value should actually be. See AprilTagFieldLayout.getOrigin
    // For now, the X axis position (forward/backward) is calculated as though the blue
    // alliance wall right hand side is the originiation point, so, the distance from
    // the blue alliance wall, to the blue alliance hub center point, plus
    // the distance between the center of the blue alliance hub and the center of
    // the red alliance hub
    public static final Pose2d kRedHubLocation = new Pose2d(
        Units.inchesToMeters(182.11 + 143.5 * 2),
        Units.inchesToMeters(158.84),
        Rotation2d.fromDegrees(0)
    );

}
