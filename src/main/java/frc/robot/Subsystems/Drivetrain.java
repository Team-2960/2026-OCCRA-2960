package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.jni.ConfigJNI;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class Drivetrain extends SubsystemBase {

    private final SparkMax lfMotor;
    private final SparkMax lbMotor;
    private final SparkMax rfMotor;
    private final SparkMax rbMotor;

    private final RelativeEncoder lEncoder; // Left Drive Encoder
    private final RelativeEncoder rEncoder; // Right Drive Encoder
    private final Distance wheelRadius;
    private final double driveRatio;

    private final SysIdRoutine sysIdRoutine;

    public final Command sysIdCommandUpQuasi;
    public final Command sysIdCommandDownQuasi;
    public final Command sysIdCommandUpDyn;
    public final Command sysIdCommandDownDyn;
    public final Command sysIdCommandGroup;

    private final DifferentialDrive diffDrive;

    // Feedback Controller
    private final PIDController drivePID;
    private final SimpleMotorFeedforward driveFF;
    private final PIDController anglePID;

    private Distance startDistanceL = Meters.of(0);
    private Distance startDistanceR = Meters.of(0);
    private Angle startAngle = Rotations.zero();

    private Pigeon2 pigeon2;

    private final DifferentialDriveKinematics kinematics;
    //private final DifferentialDrivePoseEstimator poseEstimator;

    public Drivetrain(int lfMotorID, int lbMotorID, int rfMotorID, int rbMotorID, double driveRatio,
            Distance wheelRadius) {

        lfMotor = new SparkMax(lfMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        lbMotor = new SparkMax(lbMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        rfMotor = new SparkMax(rfMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        rbMotor = new SparkMax(rbMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

        this.wheelRadius = wheelRadius;

        this.driveRatio = driveRatio;

        double distPerRev = wheelRadius.in(Meters) * Math.PI * driveRatio;

        diffDrive = new DifferentialDrive(lfMotor, rfMotor);
        diffDrive.setSafetyEnabled(false);

        SparkMaxConfig lfConfig = new SparkMaxConfig();
        SparkMaxConfig lbConfig = new SparkMaxConfig();
        SparkMaxConfig rfConfig = new SparkMaxConfig();
        SparkMaxConfig rbConfig = new SparkMaxConfig();

        rbConfig.follow(rfMotor);
        lbConfig.follow(lfMotor);

        lfMotor.configure(lfConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        lbMotor.configure(lbConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rfMotor.configure(rfConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rbMotor.configure(rbConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        lEncoder = lfMotor.getEncoder();
        rEncoder = rfMotor.getEncoder();

        lEncoder.setPosition(0);
        rEncoder.setPosition(0);

        drivePID = new PIDController(0, 0, 0);
        driveFF = new SimpleMotorFeedforward(0, 0, 0);
        anglePID = new PIDController(0, 0, 0);

        anglePID.enableContinuousInput(-180, 180);

        anglePID.setTolerance(1);

        sysIdRoutine = new SysIdRoutine(
                new Config(
                        Volts.per(Second).of(.5),
                        Volts.of(2),
                        Seconds.of(4)),
                new Mechanism(
                        volts -> driveRLMotor(volts, volts),
                        this::sysIDLogging,
                        this));

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

        kinematics = new DifferentialDriveKinematics(0);
        //poseEstimator = new DifferentialDrivePoseEstimator(kinematics, null, 0, 0, new Pose2d());

    }

    public void driveRMotor(Voltage voltage) {
        rfMotor.setVoltage(voltage);
    }

    public void driveLMotor(Voltage voltage) {
        lfMotor.setVoltage(voltage);
    }

    public void setLRate(LinearVelocity rate) {
        double pidVolt = drivePID.calculate(getLRate().in(MetersPerSecond));
        double ffVolt = driveFF.calculate(rate.in(MetersPerSecond));
        double result = MathUtil.clamp(pidVolt + ffVolt, -12, 12);

        driveLMotor(Volts.of(result));
    }

    public void setRRate(LinearVelocity rate) {
        double pidVolt = drivePID.calculate(getRRate().in(MetersPerSecond));
        double ffVolt = driveFF.calculate(rate.in(MetersPerSecond));

        driveRMotor(Volts.of(pidVolt + ffVolt));
    }

    public void setRLRate(LinearVelocity left, LinearVelocity right) {

    }

    public void setDrive(Voltage left, Voltage right) {
        lfMotor.setVoltage(left);
        rfMotor.setVoltage(right);
    }

    public void driveRLMotor(Voltage rVoltage, Voltage lVoltage) {
        rfMotor.setVoltage(rVoltage);
        lfMotor.setVoltage(lVoltage);
    }

    public void setTankDrive(double leftStick, double rightStick) {
        diffDrive.tankDrive(leftStick, rightStick);
        diffDrive.feed();
    }

    public Command getDriveRLMotorCmd(Supplier<Voltage> rVoltage, Supplier<Voltage> lVoltage) {
        return this.runEnd(
                () -> driveRLMotor(rVoltage.get(), lVoltage.get()),
                () -> driveRLMotor(Volts.zero(), Volts.zero()));
    }

    public Command getTankDriveCmd(Supplier<Double> leftStick, Supplier<Double> rightStick) {
        return this.runEnd(
                () -> setTankDrive(leftStick.get(), rightStick.get()),
                () -> setDrive(Volts.zero(), Volts.zero()));
    }

    public Command getDriveDistanceCmd(Voltage volts, Distance left, Distance right) {

        boolean leftBackwards = left.magnitude() < 0;
        boolean rightBackwards = right.magnitude() < 0;

        return this.runOnce(() -> {
            setStartDistanceR(getRightDistance());
            setStartDistanceL(getLeftDistance());
        })
        .andThen(
            this.runEnd(
                    () -> setDrive(Volts.of(volts.abs(Volts) * (leftBackwards ? -1 : 1)), Volts.of(volts.abs(Volts) * (leftBackwards ? -1 : 1))),
                    () -> driveRLMotor(Volts.of(0), Volts.of(0))
                )
                .until(
                    () -> (rightBackwards ? right.in(Meters) >= (getRightDistance().in(Meters)
                                    - this.startDistanceR.in(Meters)) : right.in(Meters) <= (getRightDistance().in(Meters)
                                    - this.startDistanceR.in(Meters)))
                                && 
                                (leftBackwards ? left.in(Meters) >= (getLeftDistance().in(Meters)
                                    - this.startDistanceL.in(Meters)) : left.in(Meters) <= (getLeftDistance().in(Meters)
                                    - this.startDistanceL.in(Meters)))
                )
        );
    }

    public Angle getRightRotations() {
        return Rotations.of(rEncoder.getPosition());
    }

    public Angle getLeftRotations() {
        return Rotations.of(lEncoder.getPosition());
    }

    public Distance getRightDistance() {
        return Meters.of(getRightRotations().in(Rotations) * driveRatio * 2 * wheelRadius.in(Meters) * Math.PI);
    }

    public Distance getLeftDistance() {
        return Meters.of(getLeftRotations().in(Rotations) * driveRatio * 2 * wheelRadius.in(Meters) * Math.PI);
    }

    

    // public Pose2d getPose() {
    //     return poseEstimator.getEstimatedPosition();
    // }

    public DifferentialDriveWheelPositions getDriveWheelPositions() {
        return new DifferentialDriveWheelPositions(getLeftDistance(), getRightDistance());
    }

    public DifferentialDriveWheelSpeeds getDriveWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLRate(), getRRate());
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getDriveWheelSpeeds());
    }

    // public void resetPose(Pose2d pose) {
    //     poseEstimator.resetPose(pose);
    // }

    private void setStartDistanceL(Distance startDistance) {
        this.startDistanceL = startDistance;
    }

    private void setStartDistanceR(Distance startDistance) {
        this.startDistanceR = startDistance;
    }

    public void setStartAngle(Angle startAngle) {
        this.startAngle = startAngle;
    }

    public Voltage getLVoltage() {
        return Voltage.ofBaseUnits(lfMotor.getAppliedOutput() * lfMotor.getBusVoltage(), Volts);
    }

    public Current getLCurrent() {
        return Amps.of(lfMotor.getOutputCurrent());
    }

    public LinearVelocity getLRate() {
        return MetersPerSecond.of(lEncoder.getVelocity() / 60 * driveRatio * 2 * wheelRadius.in(Meters) * Math.PI);
    }

    public LinearVelocity getRRate() {
        return MetersPerSecond.of(rEncoder.getVelocity() / 60 * driveRatio * 2 * wheelRadius.in(Meters) * Math.PI);
    }

    private void sysIDLogging(SysIdRoutineLog log) {
        log.motor("Drivetrain")
                .voltage(getLVoltage())
                .current(getLCurrent())
                .linearVelocity(getLRate())
                .linearPosition(getLeftDistance());

    }
}
