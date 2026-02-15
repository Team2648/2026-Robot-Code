package frc.robot.utilities;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A simple subsystem that can be used to test a single SparkMax and associated NEO motor
 */
public class SparkMAXTester extends SubsystemBase {
    private SparkMax spark;

    /**
     * Constructor
     * 
     * @param deviceID The CAN ID of the SparkMAX that needs testing
     */
    public SparkMAXTester(int deviceID) {
        spark = new SparkMax(deviceID, MotorType.kBrushless);
    }

    /**
     * Sets the speed of the motor
     * 
     * @param speed A method or lambda which returns a double between -1 and 1
     * @return A Command object that runs indefinitely to control motor speed
     */
    public Command setSpeed(DoubleSupplier speed) {
        return run(() -> {
            spark.set(speed.getAsDouble());
        });
    }
}
