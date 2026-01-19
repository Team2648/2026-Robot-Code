package frc.robot.constants;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

// TODO This is all hold over from 2025, does any of it need to change?
public class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

    public static final double kMaxSpeedMetersPerSecondAutoAlign = 2.5;

    public static final double kPXYController = 3.5;
    public static final double kPThetaController = 5;

    public static final double kAlignPXYController = 2;
    public static final double kAlignPThetaController = 5;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);

    public static final TrapezoidProfile.Constraints kAlignThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);

    // TODO This is a constant being managed like a static rewriteable variable
    public static RobotConfig kRobotConfig;
    public static boolean kAutoConfigOk;

    public static final PPHolonomicDriveController kPPDriveController = new PPHolonomicDriveController(
        new PIDConstants(kPXYController, 0, 0),
        new PIDConstants(kPThetaController, 0, 0)
    );

    public static final PathConstraints kOnTheFlyConstraints = new PathConstraints(
        kMaxSpeedMetersPerSecond, 
        kMaxAccelerationMetersPerSecondSquared, 
        kMaxAngularSpeedRadiansPerSecond, 
        kMaxAngularAccelerationRadiansPerSecondSquared
    );

    static {
        try {
            kRobotConfig = RobotConfig.fromGUISettings();
            kAutoConfigOk = true;
        } catch (IOException | ParseException e) {
            System.err.println("FAILED TO READ ROBOTCONFIG, WAS THE CONFIG SET UP IN PATHPLANNER?");
            e.printStackTrace();
            kAutoConfigOk = false;
        }
    }
}