package frc.robot.utilities;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.CompetitionConstants;

public class Utilities {
    public static final double kG = -9.81;
    
    /**
     * Returns the Alliance enumeration that indicates who will have the first
     * shift. Returns null if the data is not available. 
     * 
     * @return The Alliance that will have the first shift, or null if game specific data
     *  is not present
     */
    public static Alliance whoHasFirstShift() {
        String gameData = DriverStation.getGameSpecificMessage();

        if(gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B':
                    return Alliance.Red;
                case 'R':
                    return Alliance.Blue;
                default:
                    return null;
            }
        }
        
        return null;
    }

    /**
     * Returns the pose of the hub for the given alliance assigned to our robot.
     * If no alliance is assigned (which is unlikely) this method returns
     * the Blue hub pose, which is the closet to the field origin
     * 
     * @return The Pose2d object which represents the appropriate pose of the Hub
     */
    public static Pose2d getHubPose() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if(alliance.isEmpty() || alliance.get() == Alliance.Blue) {
            return CompetitionConstants.kBlueHubLocation;
        } else {
            return CompetitionConstants.kRedHubLocation;
        }
    }

    /**
     * Returns the AprilTag ID of the tag that is in the center of the hub
     * for the robot's assigned alliance. If no alliance is assigned (which is unlikely)
     * the Blue hub's center tag is returned. 
     * 
     * @return The tag ID that is in the center of the assigned alliance's hub
     */
    public static int getHubCenterAprilTagID() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if(alliance.isEmpty() || alliance.get() == Alliance.Blue) {
            return 26;
        } else {
            return 10;
        }
    }

    /**
     * A ChatGPT possible hallucination related to calcuating whether a shot is possible
     * for a given speed and change in X and Y position
     * 
     * Note that X in this scenario is the physical distance from the shooter exit to 
     * target. Y is the change in height from the shooter exit to the target height
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
