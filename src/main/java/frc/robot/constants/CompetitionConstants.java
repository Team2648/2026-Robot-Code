package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class CompetitionConstants {
    // THIS SHOULD BE FALSE DURING COMPETITION PLAY
    public static final boolean kLogToNetworkTables = true;

    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(
        AprilTagFields.kDefaultField
    );
}
