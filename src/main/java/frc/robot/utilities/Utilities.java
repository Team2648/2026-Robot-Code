package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Utilities {
    static String gameData;

    static Boolean Blue = false;

    static Boolean Red = false;

    public static final double kG = -9.81;
    
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

    /**
     * A ChatGPT possible hallucination related to calcuating whether a shot is possible
     * for a given speed and change in X and Y position
     * 
     * Note that X in this scenario is the physical distance from the shooter exit to 
     * target. Y is the change in height from the shooter exit to the target height
     * 
     * TODO Review ChatGPT's math more thoroughly, preferably with someone with fresher math skills
     * 
     * @param targetVMPS The target velocity of the shooter in Meters per Second
     * @param deltaXM The "as the crow flies" distance between the shooter exit and the target
     * @param deltaYM The height difference between the shooter exit and the target
     * @return A true or false value indicating whether the shot is possible
     */
    public static boolean shotPossible(double targetVMPS, double deltaXM, double deltaYM) {
        return Math.pow(targetVMPS, 4) >= 
            -9.81 * (-9.81 * Math.pow(deltaXM, 2) + 2 * deltaYM * Math.pow(targetVMPS, 2));
    }

    /**
     * A ChatGPT possible hallucination that calculates the angle required to make a shot for
     * a given speed and change in X and Y position
     * 
     * Note that X in this scenario is the physical disatance from the shooter exit to
     * target. Y is the change in height from the shooter exit to the target height
     * 
     * Setting softerShot to true changes the angle of attack to a soft, long range shot. False
     * makes the shot more of a lob
     * 
     * TODO Review ChatGPT's math more thoroughly, preferably with someone with fresher math skills
     * 
     * @param targetVMPS The target velocity of the shooter in Meters per Second
     * @param deltaXM The "as the crow flies" distance between the shooter exit and the target
     * @param deltaYM The height difference between the shooter exit and the target
     * @param softerShot True for a long range shot, false for a short range lob
     * @return The angle required of the shooter to make the shot described by the input parameters
     */
    public static double shotAngle(double targetVMPS, double deltaXM, double deltaYM, boolean softerShot) {
        double vPow2 = Math.pow(targetVMPS, 2);
        double vPow4 = Math.pow(targetVMPS, 4);
        double sqrtPiece = Math.sqrt(vPow4 - kG * (kG * Math.pow(deltaXM, 2) + 2 * deltaYM * vPow2));

        if(softerShot) {
            return Math.atan((vPow2 - sqrtPiece) / (kG * deltaXM));
        } else {
            return Math.atan((vPow2 + sqrtPiece) / (kG * deltaXM));
        }
    }
}
