package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VL53L0X;

public class RangeSensor extends SubsystemBase {
    
    private final VL53L0X distanceSensor;

    public RangeSensor(){
        distanceSensor = new VL53L0X(I2C.Port.kMXP);
    }

    public Distance getDist(){
    int disancemm = distanceSensor.getDistanceMillimeters();
    return Millimeters.of(disancemm);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Range Sensor (In)", distanceSensor.getDistanceMillimeters()/25.4);
    }
}
