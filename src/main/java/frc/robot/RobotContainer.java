// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_DrivetrainSubsystem;
  private final OI m_oi;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    // configureButtonBindings();

    m_DrivetrainSubsystem = DrivetrainSubsystem.getInstance();
    m_oi = OI.getInstance(); // Make sure OI gets initialized here, this should be the first call to getInstance()

    // Set up a default command to ensure the robot drives by default
    m_DrivetrainSubsystem.setDefaultCommand(new SwerveDriveCommand());
  }

  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  // private void configureButtonBindings() {}

  // /**
  //  * Use this to pass the autonomous command to the main {@link Robot} class.
  //  *
  //  * @return the command to run in autonomous
  //  */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return m_autoCommand;
  // }


public void setupSmartDashboardTestMode() {
  SmartDashboard.putNumber("Front Left Drive", 0.0);
  SmartDashboard.putNumber("Front Left Angle", 0.0);
  SmartDashboard.putNumber("Front Right Drive", 0.0);
  SmartDashboard.putNumber("Front Right Angle", 0.0);
  SmartDashboard.putNumber("Back Left Drive", 0.0);
  SmartDashboard.putNumber("Back Left Angle", 0.0);
  SmartDashboard.putNumber("Back Right Drive", 0.0);
  SmartDashboard.putNumber("Back Right Angle", 0.0);
}


public void testAllSystems(){
  m_DrivetrainSubsystem.setMotor(10 ,SmartDashboard.getNumber("Front Left Drive",0.0 ));
  m_DrivetrainSubsystem.setMotor(1 ,SmartDashboard.getNumber("Front Left Angle", 0.0));
  m_DrivetrainSubsystem.setMotor(2 ,SmartDashboard.getNumber("Front Right Drive", 0.0));
  m_DrivetrainSubsystem.setMotor(3 ,SmartDashboard.getNumber("Front Right Angle", 0.0));
  m_DrivetrainSubsystem.setMotor(4 ,SmartDashboard.getNumber("Back Left Drive", 0.0));
  m_DrivetrainSubsystem.setMotor(5 ,SmartDashboard.getNumber("Back Left Angle", 0.0));
  m_DrivetrainSubsystem.setMotor(6 ,SmartDashboard.getNumber("Back Right Drive", 0.0));
  m_DrivetrainSubsystem.setMotor(7 ,SmartDashboard.getNumber("Back Right Angle", 0.0));

}

}
