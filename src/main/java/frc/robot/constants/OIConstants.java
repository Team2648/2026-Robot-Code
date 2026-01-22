package frc.robot.constants;

import edu.wpi.first.wpilibj.util.Color;

public class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    
    public static final double kDriveDeadband = .01;

    public static final double kJoystickExponential = 3;

    public static final String kTeleopTab = "Teleoperated";

    public static final String kCurrentActiveHub = "Alliance Hub Currently Active";

    public static final String[] kRedBlueDisplay = {
        Color.kRed.toHexString(),
        Color.kBlue.toHexString()
    };

    public static final String[] kRedDisplay = {
        Color.kRed.toHexString()
    };

    public static final String[] kBlueDisplay = {
        Color.kBlue.toHexString()
    };
}
