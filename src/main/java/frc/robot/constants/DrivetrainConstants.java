package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {
    // TODO Hold over from 2025, adjust?
    public static final double kMaxSpeedMetersPerSecond = 4.125;
    public static final double kMaxAngularSpeed = 2 * Math.PI;

    public static final double kTrackWidth = Units.inchesToMeters(23.75);
    public static final double kWheelBase = Units.inchesToMeters(18.75);
 
    public static final double kFrontLeftMagEncoderOffset = 2.98;
    public static final double kFrontRightMagEncoderOffset = 1.18;
    public static final double kRearLeftMagEncoderOffset = 3.69;
    public static final double kRearRightMagEncoderOffset = 2.54;

    public static final int kFrontLeftDrivingCANID = 0;
    public static final int kFrontRightDrivingCANID = 3;
    public static final int kRearLeftDrivingCANID = 1;
    public static final int kRearRightDrivingCANID = 2;

    public static final int kFrontLeftTurningCANID = 8;
    public static final int kFrontRightTurningCANID = 9;
    public static final int kRearLeftTurningCANID = 7;
    public static final int kRearRightTurningCANID = 6;

    public static final int kFrontLeftAnalogInPort = 3;
    public static final int kFrontRightAnalogInPort = 2;
    public static final int kRearLeftAnalogInPort = 0;
    public static final int kRearRightAnalogInPort = 1;

    public static final boolean kGyroReversed = true;

    // TODO Hold over from 2025, adjust?
    public static final double kHeadingP = .1;
    public static final double kXTranslationP = .5;
    public static final double kYTranslationP = .5;

    // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A NEW CONFIGURATION ITEM
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );
}
