package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {
    // TODO Hold over from 2025, adjust?
    public static final double kMaxSpeedMetersPerSecond = 4.125;
    public static final double kMaxAngularSpeed = 2 * Math.PI;

    public static final double kTrackWidth = Units.inchesToMeters(23.75);
    public static final double kWheelBase = Units.inchesToMeters(18.75);
 
    public static final double kFrontLeftMagEncoderOffset = 2.965;
    public static final double kFrontRightMagEncoderOffset = 1.120;
    public static final double kRearLeftMagEncoderOffset = 3.761;
    public static final double kRearRightMagEncoderOffset = 2.573;

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

    // TODO How much do we trust gyro and encoders vs vision estimates.
    // NOTE: Bigger values indicate LESS trust. Generally all three values for a given matrix should be the same
    public static final Matrix<N3, N1> kSensorFusionOdometryStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    public static final Matrix<N3, N1> kVisionOdometryStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);

    // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A NEW CONFIGURATION ITEM
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );
}
