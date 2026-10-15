package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkParameters;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;;

public class Indexer extends SubsystemBase {

    private final SparkMax indexMotor;
    private final SparkFlexConfig indexerConfig;
    private final PIDController indexerPID;
    private final SimpleMotorFeedforward indexerFF;
    private final RelativeEncoder relEncoder;
    private final SysIdRoutine sysIdRoutine;

    public final Command sysIdCommandUpQuasi;
    public final Command sysIdCommandDownQuasi;
    public final Command sysIdCommandUpDyn;
    public final Command sysIdCommandDownDyn;
    public final Command sysIdCommandGroup;

    public Indexer(int indexerMotorID) {
        indexMotor = new SparkMax(indexerMotorID, MotorType.kBrushless);

        indexerPID = new PIDController(0, 0, 0);

        indexerFF = new SimpleMotorFeedforward(indexerMotorID, indexerMotorID, indexerMotorID);

        relEncoder = indexMotor.getEncoder();
        
        indexerConfig = new SparkFlexConfig();
        indexerConfig.idleMode(IdleMode.kBrake);
        indexerConfig.inverted(false);
        indexMotor.configure(indexerConfig, com.revrobotics.ResetMode.kNoResetSafeParameters , PersistMode.kPersistParameters);

        sysIdRoutine = new SysIdRoutine(
                new Config(
                        Volts.per(Second).of(.5),
                        Volts.of(7),
                        Seconds.of(10)),
                new Mechanism(
                        this::setVoltage,
                        this::sysIDLogging,
                         this,
                          getName()));

        sysIdCommandUpQuasi = sysIdRoutine
                .quasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward);
        sysIdCommandDownQuasi = sysIdRoutine
                .quasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse);
        sysIdCommandUpDyn = sysIdRoutine.dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward);
        sysIdCommandDownDyn = sysIdRoutine
                .dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse);

        sysIdCommandGroup = new SequentialCommandGroup(
                sysIdCommandUpQuasi,
                sysIdCommandDownQuasi,
                sysIdCommandUpDyn,
                sysIdCommandDownDyn);
    }

    public Voltage getIndexVoltage(){
        return Voltage.ofBaseUnits(indexMotor.getAppliedOutput() * indexMotor.getBusVoltage(), Volts);
    }

    public Current getIndexCurrent(){
        return Amps.of(indexMotor.getOutputCurrent());
    }

    public AngularVelocity getRate(){
        return AngularVelocity.ofBaseUnits(relEncoder.getVelocity()/60, RotationsPerSecond);
    }

    public Angle getIndexerPos(){
        return Angle.ofBaseUnits(relEncoder.getPosition(), Rotations);
    }

    private void sysIDLogging(SysIdRoutineLog log) {
        log.motor("Indexer")
                .voltage(getIndexVoltage())
                .current(getIndexCurrent())
                .angularVelocity(getRate())
                .angularPosition(getIndexerPos());
    }

    public void setVoltage(Voltage voltage) {
        indexMotor.setVoltage(voltage);
    }

    public Command getIndexCmd(Supplier<Voltage> voltage) {
        return this.runEnd(
                () -> setVoltage(voltage.get()),
                () -> setVoltage(Volts.zero()));
    }
}