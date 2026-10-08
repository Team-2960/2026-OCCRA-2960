package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{

    public final SparkMax shooterMotor;

    public Shooter(int ShooterMotorID) {

        shooterMotor = new SparkMax(ShooterMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    SparkMaxConfig shooterConfig = new SparkMaxConfig(); 

    shooterMotor.configure(shooterConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void driveShooterMotor(Voltage shooterVoltage){
        shooterMotor.setVoltage(shooterVoltage);
    }
    
    public Command getDriveShooterCmd(Supplier<Voltage> shooterVoltage){
        return this.runEnd(
            () -> driveShooterMotor(shooterVoltage.get()),
            () -> driveShooterMotor(Volts.zero())
        );
    }
    } 

