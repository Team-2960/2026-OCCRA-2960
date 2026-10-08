// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class RobotContainer {

  private final Drivetrain drivetrain;

  //private final Intake intake;

  //private final Shooter shooter;

  private final CommandXboxController driverCtrl;

  private final CommandXboxController operatorCtrl;

  public RobotContainer() {

    drivetrain = new Drivetrain(1, 2, 3, 8, 5.95, Inches.of(3));

     //intake = new Intake(5);

     //shooter = new Shooter(6);
    
    driverCtrl = new CommandXboxController(0);
    operatorCtrl = new CommandXboxController(1);

    configureBindings();
  }

  private void configureBindings() {
    // driverCtrl.axisGreaterThan(1, .05).or(() -> Math.abs(driverCtrl.getRightX()) >= 0.1)
    // .whileTrue(
    //   drivetrain.getDriveRLMotorCmd(
    //     () -> Volts.of(-MathUtil.applyDeadband(driverCtrl.getRightY() * -12, 0.05)), 
    //     () -> Volts.of(-MathUtil.applyDeadband(driverCtrl.getLeftY() * -12, 0.05))
    //   )
    // );

    driverCtrl.axisGreaterThan(1, 0.1).or(() -> Math.abs(driverCtrl.getRightY()) >= 0.1)
            .onTrue(
                drivetrain.getTankDriveCmd(() -> -driverCtrl.getLeftY(), () -> -driverCtrl.getRightY()));
     //operatorCtrl.x().whileTrue(intake.getIntakeMotorCmd(() -> Volts.of(6)));

     //operatorCtrl.rightTrigger(.1).whileTrue(shooter.getDriveShooterCmd(() -> Volts.of(12)));


  }

  private Command getTestAuton(){
    return Commands.sequence(
      drivetrain.getDriveDistanceCmd(Volts.of(4.5), Feet.of(2), Feet.of(8.5)
    ));
  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
