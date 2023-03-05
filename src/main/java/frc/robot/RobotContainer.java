// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import Auto.Auto;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private DriveBase driveBase;
  private final XboxController operatorController;
  Intake intake;
  Arm arm;
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick driverJoystick =
      new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private TankDrive driveCommand;
  public RobotContainer() {
    
    driveBase = new DriveBase();
    
    driveCommand = new TankDrive(driveBase);
    driveBase.setDefaultCommand(driveCommand);
    operatorController = new XboxController(1);
    intake = new Intake();
    arm = new Arm();
    // Configure the trigger bindings
    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
    Trigger intakeIn =  new Trigger(()->operatorController.getRawButton(4));
    intakeIn.whileTrue(new InstantCommand(()->intake.setIntake(1)));
    intakeIn.onFalse(new InstantCommand(()->intake.setIntake(0)));
    
    Trigger intakeOut =  new Trigger(()->operatorController.getRawButton(6));
    intakeOut.whileTrue(new InstantCommand(()->intake.setIntake(-1)));
    intakeOut.onFalse(new InstantCommand(()->intake.setIntake(0)));
    //#TODO make more poses
    Trigger armHome = new Trigger(()->operatorController.getRawButton(7));
    armHome.onTrue(new InstantCommand(()->arm.setArm(0)));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
   public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Auto(driveBase);
    //return null;
  } 
}
