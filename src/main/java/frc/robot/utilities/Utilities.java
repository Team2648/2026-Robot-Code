package frc.robot.utilities;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Utilities {
    static String gameData;

    static Boolean Blue = false;

    static Boolean Red = false;
    
    public static Alliance ShiftFirst() {
        gameData = DriverStation.getGameSpecificMessage();


        if(gameData.length() > 0) {
            switch (gameData.charAt(0)) {
            case 'B' :
                return Alliance.Red;
            case 'R' :
                return Alliance.Blue;
            default :
            return null;
            }
        }
        return null;
    }

}
