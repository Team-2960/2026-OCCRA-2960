package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class Shooter extends SubsystemBase {

    public final SparkMax shooterMotor;
    private final PIDController shooterPID;
    private final SimpleMotorFeedforward shooterFF;
    private final RelativeEncoder relEncoder;
    private final SysIdRoutine sysIdRoutine;

    public final Command sysIdCommandUpQuasi;
    public final Command sysIdCommandDownQuasi;
    public final Command sysIdCommandUpDyn;
    public final Command sysIdCommandDownDyn;
    public final Command sysIdCommandGroup;

    public Shooter(int ShooterMotorID) {

        shooterMotor = new SparkMax(ShooterMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig shooterConfig = new SparkMaxConfig();

        shooterPID = new PIDController(2, 0, 0.1);

        shooterFF = new SimpleMotorFeedforward(0, 0, 0);

        relEncoder = shooterMotor.getEncoder();

        shooterMotor.configure(shooterConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

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

    public AngularVelocity getShooterRPM(){
        return AngularVelocity.ofBaseUnits(relEncoder.getVelocity(), Rotations.per(Minute));
    }

    public Voltage getIndexVoltage(){
        return Voltage.ofBaseUnits(shooterMotor.getAppliedOutput() * shooterMotor.getBusVoltage(), Volts);
    }

    public Current getIndexCurrent(){
        return Amps.of(shooterMotor.getOutputCurrent());
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

    public void setVoltage(Voltage shooterVoltage) {
        shooterMotor.setVoltage(shooterVoltage);
    }

    public void setRate(AngularVelocity rate) {
        double pidVolt = shooterPID.calculate(getRate().in(RotationsPerSecond), rate.in(RotationsPerSecond));
        double ffVolt = shooterFF.calculate(rate.in(RadiansPerSecond));

        setVoltage(Volts.of(pidVolt));
    }

    public Command getSysIdCommandGroup(){
        return sysIdCommandGroup;
    }

    public Command getShooterVoltCmd(Supplier<Voltage> shooterVoltage) {
        return this.runEnd(
                () -> setVoltage(shooterVoltage.get()),
                () -> setVoltage(Volts.zero()));
    }

     public Command getShooterPIDCmd(AngularVelocity shooterVelocity) {
        return this.runEnd(
                () -> setRate(shooterVelocity),
                () -> setVoltage(Volts.zero()));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("In Voltage", shooterMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter RPM", getShooterRPM().in(Rotations.per(Second)));
    }

}
