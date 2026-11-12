// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.CommandsManager;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.IntakeRoller;
import frc.robot.Subsystems.RangeSensor;
import frc.robot.Subsystems.Shooter;


public class RobotContainer {

    private final Drivetrain drivetrain;

    private final Intake intake;

    private final IntakeRoller intakeRoller;

    private final Indexer indexer;

    private final Shooter shooter;

    private final RangeSensor rangeSensor;

    private final CommandsManager commandsManager;

    private final CommandXboxController driverCtrl;

    private final CommandXboxController operatorCtrl;

    private final SendableChooser<Command> autonChooser;

    public RobotContainer() {

        drivetrain = new Drivetrain(1, 2, 3, 4, .168067226891, Inches.of(3), 15);

        intake = new Intake(5);

        indexer = new Indexer(7);

        shooter = new Shooter(8);

        intakeRoller = new IntakeRoller(6, CANBus.roboRIO(), .6);

        rangeSensor = new RangeSensor();

        commandsManager = new CommandsManager(shooter, indexer, intake);

        driverCtrl = new CommandXboxController(0);
        operatorCtrl = new CommandXboxController(1);
        
        autonChooser = new SendableChooser<>();
        autonChooser.setDefaultOption("None", Commands.print("No autonomous command configured"));
        //autonChooser.addOption("Left Auto 8Pts", left8PointAutonImprove());
        //autonChooser.addOption("Right Auto 8Pts", right8PointAutonImprove());
        //autonChooser.addOption("TestIntake", TestIntake());
        autonChooser.addOption("MoveAuton", AutonMove());
        autonChooser.addOption("center4ball", center4ball());
        autonChooser.addOption("LeftAuton", Left4Ball());
        

        configureBindings();

        SmartDashboard.putData("Auton Chooser", autonChooser);
    }

    private void configureBindings() {

        driverCtrl.axisGreaterThan(1, 0.1)
                .or(() -> Math.abs(driverCtrl.getRightY()) >= 0.1)
                .onTrue(
                        drivetrain.getTankDriveCmd(
                                () -> driverCtrl.getHID().getLeftBumperButton() ? -driverCtrl.getLeftY() * .4
                                        : -driverCtrl.getLeftY() * 0.75,
                                () -> driverCtrl.getHID().getLeftBumperButton() ? -driverCtrl.getRightY() * .4
                                        : -driverCtrl.getRightY() * 0.75));

        operatorCtrl.x().whileTrue(indexer.getIndexCmd(() -> Volts.of(-4)));

        operatorCtrl.leftTrigger(.1).whileTrue(intake.getIntakeMotorCmd(() -> Volts.of
        (-12)));
        
        BooleanSupplier bButtonSupplier = operatorCtrl.b();

        operatorCtrl.leftBumper().whileTrue(intake.getIntakeMotorCmd(() -> Volts.of(4.8)));
    
        operatorCtrl.rightBumper().whileTrue(shooter.getShooterVoltCmd(() -> Volts.of(3.5)));

        operatorCtrl.rightTrigger(.1).whileTrue(commandsManager.highAutoIndexShotCmd(bButtonSupplier));

        operatorCtrl.povLeft().whileTrue(ShooterBackupLeft());

        operatorCtrl.povRight().whileTrue(ShooterBackupRight());

        operatorCtrl.povDown().whileTrue(ShooterBackup());

        operatorCtrl.leftTrigger(.1).whileTrue(intakeRoller.intakeInCmd());

        operatorCtrl.leftBumper().whileTrue(intakeRoller.intakeOutCmd());

        operatorCtrl.rightBumper().whileTrue(shooter.getSpeedShotCmd(RPM.of(1940), RPM.of(100), RPM.of(100)));

        driverCtrl.a().onTrue(shooter.getSysIdCommandGroup());

        driverCtrl.x().whileTrue(drivetrain.getResetPigeonCmd());


     //   operatorCtrl.rightTrigger(.1).whileTrue(commandsManager.highAutoIndexShotCmd());

       // operatorCtrl.rightBumper().whileTrue(commandsManager.lowAutoIndexShotCmd());
    }

     public Command AutonMove() { // this is our longer, 8 point 4 ball auton, with greater efficenty.
        return Commands.sequence(
        drivetrain.getDriveDistanceCmd(Volts.of(2), Feet.of(-3), Feet.of(-3))
        );
     }

public Command ShooterBackup() { // this is our longer, 8 point 4 ball auton, with greater efficenty.
        return Commands.sequence(
        drivetrain.getDriveDistanceCmd(Volts.of(3), Inches.of(3), Inches.of(3))
        );
}

public Command ShooterBackupRight() { // this is our longer, 8 point 4 ball auton, with greater efficenty.
        return Commands.sequence(
        drivetrain.getDriveDistanceCmd(Volts.of(4), Inches.of(2.5), Inches.of(2.5)),

        drivetrain.getDriveToAnglePIDCmd(Degrees.of(-20),Degrees.of(3))
        );
}

public Command ShooterBackupLeft() { // this is our longer, 8 point 4 ball auton, with greater efficenty.
        return Commands.sequence(
        drivetrain.getDriveDistanceCmd(Volts.of(4), Inches.of(2.5), Inches.of(2.5)),

        drivetrain.getDriveToAnglePIDCmd(Degrees.of(20),Degrees.of(3))
        );
}

public Command TestIntake(){
        return Commands.race(
                Commands.waitSeconds(4),
                intakeRoller.intakeInCmd(),
                intake.getIntakeMotorCmd(() -> Volts.of(3)));
}
 public Command Right4Ball(){
        return Commands.sequence(
                Commands.race(drivetrain.getDriveDistanceCmd(Volts.of(3), Inches.of(-63), Inches.of(-59)),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                 Commands.race(drivetrain.getDriveToAnglePIDCmd(Degrees.of(-30),Degrees.of(3)),
                 shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                Commands.race(Commands.waitSeconds(.2),
                        indexer.getIndexCmd(() ->Volts.of(3)),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                Commands.race(Commands.waitSeconds(.8),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                 Commands.race(Commands.waitSeconds(1),
                        indexer.getIndexCmd(() ->Volts.of(3)),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(30),Degrees.of(3)),

                Commands.race(drivetrain.getDriveDistanceCmd(Volts.of(3), Inches.of(139), Inches.of(141)),
                        intake.getIntakeMotorCmd(() -> Volts.of(3)),
                        intakeRoller.intakeInCmd()),

                 Commands.race(Commands.waitSeconds(1),
                        intake.getIntakeMotorCmd(() -> Volts.of(3)),
                        intakeRoller.intakeInCmd()),

                Commands.race(drivetrain.getDriveDistanceCmd(Volts.of(3), Inches.of(-131), Inches.of(-135)),
                         intake.getIntakeMotorCmd(() -> Volts.of(3)),
                         intakeRoller.intakeInCmd(),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                 Commands.race(drivetrain.getDriveToAnglePIDCmd(Degrees.of(-30),Degrees.of(3)),
                 shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                Commands.race(Commands.waitSeconds(.2),
                        indexer.getIndexCmd(() ->Volts.of(3)),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                Commands.race(Commands.waitSeconds(.8),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                 Commands.race(Commands.waitSeconds(1),
                        indexer.getIndexCmd(() ->Volts.of(3)),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5)))
                
                );
}

        public Command Left4Ball(){
        return Commands.sequence(
                Commands.race(drivetrain.getDriveDistanceCmd(Volts.of(3), Inches.of(-63), Inches.of(-59)),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                 Commands.race(drivetrain.getDriveToAnglePIDCmd(Degrees.of(-30),Degrees.of(3)),
                 shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                Commands.race(Commands.waitSeconds(.2),
                        indexer.getIndexCmd(() ->Volts.of(3)),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                Commands.race(Commands.waitSeconds(.8),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                 Commands.race(Commands.waitSeconds(1),
                        indexer.getIndexCmd(() ->Volts.of(3)),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(30),Degrees.of(3)),

                Commands.race(drivetrain.getDriveDistanceCmd(Volts.of(3), Inches.of(139), Inches.of(141)),
                        intake.getIntakeMotorCmd(() -> Volts.of(3)),
                        intakeRoller.intakeInCmd()),

                 Commands.race(Commands.waitSeconds(1),
                        intake.getIntakeMotorCmd(() -> Volts.of(3)),
                        intakeRoller.intakeInCmd()),

                Commands.race(drivetrain.getDriveDistanceCmd(Volts.of(3), Inches.of(-131), Inches.of(-135)),
                         intake.getIntakeMotorCmd(() -> Volts.of(3)),
                         intakeRoller.intakeInCmd(),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                 Commands.race(drivetrain.getDriveToAnglePIDCmd(Degrees.of(-30),Degrees.of(3)),
                 shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                Commands.race(Commands.waitSeconds(.2),
                        indexer.getIndexCmd(() ->Volts.of(3)),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                Commands.race(Commands.waitSeconds(.8),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                 Commands.race(Commands.waitSeconds(1),
                        indexer.getIndexCmd(() ->Volts.of(3)),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5)))
                
                );
}

    public Command center4ball(){
        return Commands.sequence(
                Commands.race(drivetrain.getDriveDistanceCmd(Volts.of(3), Inches.of(-66), Inches.of(-62)),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                Commands.race(Commands.waitSeconds(.5),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                Commands.race(Commands.waitSeconds(.2),
                        indexer.getIndexCmd(() ->Volts.of(3)),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                Commands.race(Commands.waitSeconds(.8),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                 Commands.race(Commands.waitSeconds(1),
                        indexer.getIndexCmd(() ->Volts.of(3)),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                 Commands.race(drivetrain.getDriveDistanceCmd(Volts.of(3), Inches.of(142), Inches.of(144)),
                        intake.getIntakeMotorCmd(() -> Volts.of(3)),
                        intakeRoller.intakeInCmd()),

                 Commands.race(Commands.waitSeconds(1),
                        intake.getIntakeMotorCmd(() -> Volts.of(3)),
                        intakeRoller.intakeInCmd()),

                Commands.race(drivetrain.getDriveDistanceCmd(Volts.of(3), Inches.of(-134), Inches.of(-138)),
                         intake.getIntakeMotorCmd(() -> Volts.of(3)),
                         intakeRoller.intakeInCmd(),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                Commands.race(Commands.waitSeconds(.2),
                        indexer.getIndexCmd(() ->Volts.of(3)),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                Commands.race(Commands.waitSeconds(.8),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5))),

                 Commands.race(Commands.waitSeconds(1),
                        indexer.getIndexCmd(() ->Volts.of(3)),
                        shooter.getShooterVoltCmd(() -> Volts.of(3.5)))

        );
}
    public Command right8PointAutonImprove() { // this is our longer, 8 point 4 ball auton, with greater efficenty.
        return Commands.sequence(

               Commands.race(drivetrain.getDriveDistanceCmd(Volts.of(3), Inches.of(-25), Inches.of(-25)),
                shooter.getShooterVoltCmd(() -> Volts.of(5))),

                Commands.race(drivetrain.getDriveToAnglePIDCmd(Degrees.of(-41.6), Degrees.of(3)),
                 shooter.getShooterVoltCmd(() -> Volts.of(5))),

                Commands.race(
                        Commands.waitSeconds(3),
                        indexer.getIndexCmd(() ->Volts.of(3)),
                        shooter.getShooterVoltCmd(() -> Volts.of(5))),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(41.6), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(3), Feet.of(7.5), Feet.of(7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(3)),

                Commands.deadline(drivetrain.getDriveDistanceCmd(Volts.of(3), Inches.of(20), Inches.of(20)),
                        intake.getIntakeMotorCmd(() -> Volts.of(6)),
                        indexer.getIndexCmd(() -> Volts.of(6))),

                drivetrain.getDriveDistanceCmd(Volts.of(3), Inches.of(-10), Inches.of(-10)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-90), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(2), Feet.of(-7.5), Feet.of(-7.5)),

                Commands.race(drivetrain.getDriveToAnglePIDCmd(Degrees.of(-29), Degrees.of(3)),
                        indexer.getIndexCmd(() -> Volts.of(-3))),
                        shooter.getShooterVoltCmd(() -> Volts.of(5)),

                Commands.race(
                        Commands.waitSeconds(3),
                        indexer.getIndexCmd(() -> Volts.of(3)),
                        shooter.getShooterVoltCmd(() -> Volts.of(5))),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(29), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(3), Feet.of(7.5), Feet.of(7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(3)));
    }

    public Command right4PointAutonImprove() { // this is our longer, 4 point 4 ball auton, with greater efficenty.
        return Commands.sequence(

                drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(25), Inches.of(25)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-14.9), Degrees.of(3)),

                Commands.race(
                        Commands.waitSeconds(3),
                        indexer.getIndexCmd(() ->Volts.of(3)),
                        shooter.getShooterVoltCmd(() -> Volts.of(12))),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(14.9), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(3)),

                Commands.deadline(
                        drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(40), Inches.of(40)),
                        intake.getIntakeMotorCmd(() -> Volts.of(6)),
                        indexer.getIndexCmd(() -> Volts.of(6))),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(-10), Inches.of(-10)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-90), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(-7.5), Feet.of(-7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-8.8), Degrees.of(3)),

                Commands.race(
                        Commands.waitSeconds(.25),
                        indexer.getIndexCmd(() -> Volts.of(-3))),

                Commands.race(
                        Commands.waitSeconds(3),
                        shooter.getShooterVoltCmd(() -> Volts.of(12))),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(8.8), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(3)));
    }

    public Command left8PointAutonImprove() { // this is our longer, 8 point 4 ball auton, with greater efficenty.
        return Commands.sequence(

                drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(25), Inches.of(25)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(41.6), Degrees.of(3)),

                Commands.race(
                        Commands.waitSeconds(3),
                        indexer.getIndexCmd(() ->Volts.of(3)),
                        shooter.getShooterVoltCmd(() -> Volts.of(12))),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-41.6), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-90), Degrees.of(3)),

                Commands.deadline(
                        drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(40), Inches.of(40)),
                        intake.getIntakeMotorCmd(() -> Volts.of(6)),
                         indexer.getIndexCmd(() -> Volts.of(6))),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(-10), Inches.of(-10)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(-7.5), Feet.of(-7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(29), Degrees.of(3)),

                Commands.race(
                        Commands.waitSeconds(.25),
                        indexer.getIndexCmd(() -> Volts.of(-3))),
                        
                Commands.race(
                        Commands.waitSeconds(3),
                        shooter.getShooterVoltCmd(() -> Volts.of(12))),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-29), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-90), Degrees.of(3)));
    }

    public Command left4PointAutonImprove() { // this is our longer, 4 point 4 ball auton, with greater efficenty.
        return Commands.sequence(

                drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(25), Inches.of(25)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(14.9), Degrees.of(3)),

                Commands.race(
                        Commands.waitSeconds(3),
                        indexer.getIndexCmd(() ->Volts.of(3)),
                        shooter.getShooterVoltCmd(() -> Volts.of(12))),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-14.9), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-90), Degrees.of(3)),

                Commands.deadline(
                        drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(40), Inches.of(40)),
                        intake.getIntakeMotorCmd(() -> Volts.of(6)),
                         indexer.getIndexCmd(() -> Volts.of(6))),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Inches.of(-10), Inches.of(-10)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(-7.5), Feet.of(-7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(8.8), Degrees.of(3)),

                Commands.race(
                        Commands.waitSeconds(.25),
                        indexer.getIndexCmd(() -> Volts.of(-3))),

                Commands.race(
                        Commands.waitSeconds(3),
                        shooter.getShooterVoltCmd(() -> Volts.of(12))),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-8.8), Degrees.of(3)),

                drivetrain.getDriveDistanceCmd(Volts.of(6), Feet.of(7.5), Feet.of(7.5)),

                drivetrain.getDriveToAnglePIDCmd(Degrees.of(-90), Degrees.of(3)));
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
