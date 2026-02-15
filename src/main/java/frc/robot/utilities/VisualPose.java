package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A record class which represents the source of a visual pose, the pose itself
 * and the timestamp the pose was generated. 
 */
public record VisualPose(String cameraName, Pose2d visualPose, double timestamp) {}
