// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.OI;

public class SwerveDriveCommand extends CommandBase {
  private DrivetrainSubsystem drivetrain;
  /** Creates a new SwerveDriveCommand. */
  public SwerveDriveCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = DrivetrainSubsystem.getInstance();
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double forward = -OI.getInstance().getForward();
    
    SmartDashboard.putNumber("Forward after neg sign", forward);
    //if (Math.abs(forward) < Constants.JOYSTICK_THRESHOLD) {
    //  forward = 0;
    //}
    // deadband - In example code deadband is implemented using a method in the untilities class
    forward = Math.copySign(Math.pow(forward, 2.0), forward); // square joystick input while keeping its sign

    double strafe = -OI.getInstance().getStrafe();
    SmartDashboard.putNumber("Strafe after neg sign", strafe);

    //if (Math.abs(strafe) < Constants.JOYSTICK_THRESHOLD) {
    //  strafe = 0;
    //}
    // deadband - In example code deadband is implemented using a method in the untilities class
    strafe = Math.copySign(Math.pow(strafe, 2.0), strafe); // square joystick input while keeping its sign

    double rotation = -OI.getInstance().getRotation();
    SmartDashboard.putNumber("Rotation after neg sign", rotation);
    //if (Math.abs(rotation) < Constants.JOYSTICK_THRESHOLD) {
    //  rotation = 0;
    //}
    // deadband - In example code deadband is implemented using a method in the untilities class
    rotation = Math.copySign(Math.pow(rotation, 2.0), rotation); // square joystick input while keeping its sign

    drivetrain.drive(new Translation2d(forward, strafe), rotation, false);

    SmartDashboard.putNumber("Forward squared", forward);
    SmartDashboard.putNumber("Strafe squared", strafe);
    SmartDashboard.putNumber("Rotation squared", rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
