package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {
    // TODO Hold over from 2025, adjust?
    public static final double kMaxSpeedMetersPerSecond = 4.125;
    public static final double kMaxAngularSpeed = 2 * Math.PI;

    // TODO Replace zeros with real numbers
    public static final double kTrackWidth = Units.inchesToMeters(0);
    public static final double kWheelBase = Units.inchesToMeters(0);

    // TODO Replace zeros with real numbers
    // These values should be determinable by writing the magnetic encoder output
    // to the dashboard, and manually aligning all wheels such that the bevel gears
    // on the side of the wheel all face left. Some known straight edge (like 1x1 or similar)
    // should be used as the alignment medium. If done correctly, and the modules aren't disassembled,
    // then these values should work throughout the season the first time they are set. 
    public static final double kFrontLeftMagEncoderOffset = 0;
    public static final double kFrontRightMagEncoderOffset = 0;
    public static final double kRearLeftMagEncoderOffset = 0;
    public static final double kRearRightMagEncoderOffset = 0;

    // Kraken CAN IDs 
    // TODO Real CAN IDs
    public static final int kFrontLeftDrivingCANID = 0;
    public static final int kFrontRightDrivingCANID = 0;
    public static final int kRearLeftDrivingCANID = 0;
    public static final int kRearRightDrivingCANID = 0;

    // SparkMAX CAN IDs
    // TODO Real CAN IDs
    public static final int kFrontLeftTurningCANID = 0;
    public static final int kFrontRightTurningCANID = 0;
    public static final int kRearLeftTurningCANID = 0;
    public static final int kRearRightTurningCANID = 0;

    // Analog Encoder Input Ports 
    // TODO Real Port IDs
    public static final int kFrontLeftAnalogInPort = 0;
    public static final int kFrontRightAnalogInPort = 0;
    public static final int kRearLeftAnalogInPort = 0;
    public static final int kRearRightAnalogInPort = 0;

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
