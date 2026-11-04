package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

public class VL53L0X {
    private final I2C i2c;
    private static final int DEFAULT_ADDRESS = 0x29; // 7-bit I2C address (0x52 shifted right)

    // Device Registers
    private static final int SYSRANGE_START = 0x00;
    private static final int RESULT_RANGE_STATUS = 0x14;

    public VL53L0X(I2C.Port port) {
        // Initialize the I2C port with the sensor's 7-bit address
        i2c = new I2C(port, DEFAULT_ADDRESS);
        initSensor();
    }

    private void initSensor() {
        // Basic mandatory initialization registers for the VL53L0X chip
        i2c.write(0x88, 0x00);
        i2c.write(0x80, 0x01);
        i2c.write(0xFF, 0x01);
        i2c.write(0x00, 0x00);
        i2c.write(0x91, 0x3C); // Default tuning snippet
        i2c.write(0x00, 0x01);
        i2c.write(0xFF, 0x00);
        i2c.write(0x80, 0x00);

        // Put sensor into continuous back-to-back mode
        i2c.write(SYSRANGE_START, 0x02); 
    }

    /**
     * Reads the distance from the sensor.
     * @return Distance in millimeters, or -1 if the reading is invalid.
     */
    public int getDistanceMillimeters() {
        byte[] buffer = new byte[2];
        
        // Read 2 bytes starting from the result register (0x14)
        boolean dataReceived = !i2c.read(RESULT_RANGE_STATUS + 10, 2, buffer);
        
        if (dataReceived) {
            // Combine high byte and low byte
            int distance = ((buffer[0] & 0xFF) << 8) | (buffer[1] & 0xFF);
            
            // 8190 or 8191 usually indicates an out-of-bounds error/timeout
            if (distance > 2000 || distance <= 0) {
                return -1; 
            }
            return distance;
        }
        return -1;
    }
}
