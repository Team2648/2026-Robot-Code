package frc.robot.utilities;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMAXTester extends SubsystemBase {
    private SparkMax spark;

    public SparkMAXTester(int deviceID) {
        spark = new SparkMax(deviceID, MotorType.kBrushless);
    }

    public Command setSpeed(DoubleSupplier speed) {
        return run(() -> {
            spark.set(speed.getAsDouble());
        });
    }
}
