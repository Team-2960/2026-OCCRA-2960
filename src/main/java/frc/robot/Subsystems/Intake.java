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
    
private final SparkMax bottomIntakeMotor;

public final SparkMax topIntakeMotor;

    public Intake(int bottomIntakeMotorID, int topIntakeMotorID){

        bottomIntakeMotor = new SparkMax(bottomIntakeMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

        topIntakeMotor = new SparkMax(topIntakeMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
        SparkMaxConfig bottomIntakeConfig = new SparkMaxConfig();

        SparkMaxConfig topIntakeConfig = new SparkMaxConfig();







        bottomIntakeMotor.configure(bottomIntakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        topIntakeMotor.configure(topIntakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void driveBothIntakeMotor(Voltage intakeVoltage){
        bottomIntakeMotor.setVoltage(intakeVoltage);
        topIntakeMotor.setVoltage(intakeVoltage);
    }

    public void drivetopIntakeMotor(Voltage topIntakeVoltage){
        topIntakeMotor.setVoltage(topIntakeVoltage);
    }

    public Command getIntakeMotorCmd(Supplier<Voltage> intakeVoltage){
        return this.runEnd(
            () -> driveBothIntakeMotor(intakeVoltage.get()),
            () -> driveBothIntakeMotor(Volts.zero())
        );
    }

        public Command gettopIntakeMotorCmd(Supplier<Voltage> topIntakeVoltage){
        return this.runEnd(
            () -> drivetopIntakeMotor(topIntakeVoltage.get()),
            () -> drivetopIntakeMotor(Volts.zero())
        );
    }
}