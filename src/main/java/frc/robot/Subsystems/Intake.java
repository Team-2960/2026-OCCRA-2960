package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    
private final SparkMax intakeMotor;

    public Intake(int intakeMotorID){

        intakeMotor = new SparkMax(intakeMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
        SparkMaxConfig intakeConfig = new SparkMaxConfig();

        intakeMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
    }

    public void driveIntakeMotor(Voltage intakeVoltage){
        intakeMotor.setVoltage(intakeVoltage);
    }

    public Command getIntakeMotorCmd(Supplier<Voltage> intakeVoltage){
        return this.runEnd(
            () -> driveIntakeMotor(intakeVoltage.get()),
            () -> driveIntakeMotor(Volts.zero())
        );
    }


}
