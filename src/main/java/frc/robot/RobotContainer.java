// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.TankDriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.MathUtilExt;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final RobotPreferences config = RobotPreferences.getInstance();

  // The robot's subsystems and commands are defined here...
  public final TankDriveSubsystem tankDriveSubsystem = new TankDriveSubsystem();

  private final CommandXboxController xbox1 = new CommandXboxController(0);


  /** The container for the robot. Contains subsystems, OI devices, and commands.
   * Singleton
   * */
  private RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private static class RobotContainerHolder {
    private static final RobotContainer INSTANCE = new RobotContainer();
  }

  public static RobotContainer getInstance() {
    return RobotContainerHolder.INSTANCE;
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
    CommandScheduler.getInstance().cancelAll();

    tankDriveSubsystem.setDefaultCommand(tankDriveSubsystem.arcadeDriveCommand(
            () -> {
              var p = config.getForwardAxisPower();
              return -MathUtilExt.axisPowerCurve(xbox1.getLeftY(), p.exponent(), p.deadzone(), p.maxPower());
            },
            () -> {
              var p = config.getTurnAxisPower();
              return -MathUtilExt.axisPowerCurve(xbox1.getRightX(), p.exponent(), p.deadzone(), p.maxPower());
            }
    ));

    xbox1.y().debounce(1.0).whileTrue(tankDriveSubsystem.runGyroCalibration());
  }
}
