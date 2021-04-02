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
  SmartDashboard.putNumber("FL Drive", 0.0);
  SmartDashboard.putNumber("FL Angle", 0.0);
  SmartDashboard.putNumber("FR Drive", 0.0);
  SmartDashboard.putNumber("FR Angle", 0.0);
  SmartDashboard.putNumber("BL Drive", 0.0);
  SmartDashboard.putNumber("BL Angle", 0.0);
  SmartDashboard.putNumber("BR Drive", 0.0);
  SmartDashboard.putNumber("BR Angle", 0.0);
}


public void testAllSystems(){
  m_DrivetrainSubsystem.getFrontLeftSwerveModule().setDriveMotor(SmartDashboard.getNumber("FL Drive",0.0));
  m_DrivetrainSubsystem.getFrontLeftSwerveModule().setAngleMotor(SmartDashboard.getNumber("FL Angle", 0.0));
  m_DrivetrainSubsystem.getFrontRightSwerveModule().setDriveMotor(SmartDashboard.getNumber("FR Drive", 0.0));
  m_DrivetrainSubsystem.getFrontRightSwerveModule().setAngleMotor(SmartDashboard.getNumber("FR Angle", 0.0));
  m_DrivetrainSubsystem.getBackLeftSwerveModule().setDriveMotor(SmartDashboard.getNumber("BL Drive", 0.0));
  m_DrivetrainSubsystem.getBackLeftSwerveModule().setAngleMotor(SmartDashboard.getNumber("BL Angle", 0.0));
  m_DrivetrainSubsystem.getBackRightSwerveModule().setDriveMotor(SmartDashboard.getNumber("BR Drive", 0.0));
  m_DrivetrainSubsystem.getBackRightSwerveModule().setAngleMotor(SmartDashboard.getNumber("BR Angle", 0.0));

}

}
