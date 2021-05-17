// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SwerveDriveCommand;                                                                                                                                                                                              //Bao was here                  



public class DrivetrainSubsystem extends SubsystemBase {

  private static DrivetrainSubsystem instance;
  private ADIS16470_IMU gyroscope = new ADIS16470_IMU();

  /**Angle Offsets */
  private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(0.0); //external magnet encoder - internal motor encoder
  private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(0.0); //external magnet encoder - internal motor encoder
  private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(0.0); //external magnet encoder - internal motor encoder
  private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(0.0); //external magnet encoder - internal motor encoder

  private final SwerveModule[] swerveModules;
  private SwerveModuleState[] swerveModuleStates;
  private final SwerveModule frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule, backRightSwerveModule;
  
  /** Creation of Swerve modules relative to the robot center */
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(Constants.WHEELBASE_M/2.0, Constants.TRACKWIDTH_M/2.0),
    new Translation2d(Constants.WHEELBASE_M/2.0, -Constants.TRACKWIDTH_M/2.0),
    new Translation2d(-Constants.WHEELBASE_M/2.0, Constants.TRACKWIDTH_M/2.0),
    new Translation2d(-Constants.WHEELBASE_M/2.0, -Constants.TRACKWIDTH_M/2.0)
  );

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {

    frontLeftSwerveModule = new SwerveModule(10,1,11,Math.toRadians(120));//last 2 are random vals. They are canCoder ID and offset angle in radians
    frontRightSwerveModule = new SwerveModule(2,3,12,Math.toRadians(341));//last 2 are random vals. They are canCoder ID and offset angle in radians
    backLeftSwerveModule = new SwerveModule(4,5,13,Math.toRadians(325));//last 2 are random vals. They are canCoder ID and offset angle in radians
    backRightSwerveModule = new SwerveModule(6,7,14,Math.toRadians(269));//last 2 are random vals. They are canCoder ID and offset angle in radianss
    swerveModules = new SwerveModule[]{frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule, backRightSwerveModule};
    
    frontLeftSwerveModule.initRotationOffset();
    frontRightSwerveModule.initRotationOffset();
    backLeftSwerveModule.initRotationOffset();
    backRightSwerveModule.initRotationOffset();
  }

  public static DrivetrainSubsystem getInstance(){
    if (instance == null){
      instance = new DrivetrainSubsystem();
    }
    return instance;
  }
  
  public void drive(Translation2d translation, double rotation, boolean fieldOriented) {

    ChassisSpeeds speeds;
    // Convert the inputs from the controller into a vector of desired translational/rotational speeds for the robot
    if(fieldOriented) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(5.0*translation.getX(), 5.0*translation.getY(), 8.0*rotation, Rotation2d.fromDegrees(gyroscope.getAngle()));
    } else {
      speeds = new ChassisSpeeds(5.0*translation.getX(), 5.0*translation.getY(), 8.0*rotation);
    }

    SmartDashboard.putNumber("ChassisSpeed x (m/s)", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeed y (m/s)", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeed rot (r/s)", speeds.omegaRadiansPerSecond);

    swerveModuleStates = kinematics.toSwerveModuleStates(speeds); // calculate the states of the individual swerve modules from the global vector
    
    // Update each swerve module according to the SwerveModuleStates computed by WPILib
    for(int i = 0; i < swerveModules.length; i++){
      //swerveModules[i].setPIDFDrive(SmartDashboard.getNumber("Drive P", 0.15), SmartDashboard.getNumber("Drive I", 0.0), SmartDashboard.getNumber("Drive D", 0.0), SmartDashboard.getNumber("Drive FF", 0.0));
      //swerveModules[i].setPIDFAngle(SmartDashboard.getNumber("Angle P", 0.0), SmartDashboard.getNumber("Angle I", 0.0), SmartDashboard.getNumber("Angle D", 0.0), SmartDashboard.getNumber("Angle FF", 0.0));
      swerveModules[i].setDesiredState(swerveModuleStates[i]);
    }

  }

public void resetGyroscope() {
	gyroscope.reset();
}

public SwerveModule getFrontLeftSwerveModule(){
  return frontLeftSwerveModule;
}

public SwerveModule getFrontRightSwerveModule(){
  return frontRightSwerveModule;
}

public SwerveModule getBackLeftSwerveModule(){
  return backLeftSwerveModule;
}

public SwerveModule getBackRightSwerveModule(){
  return backRightSwerveModule;
}

public SwerveModule[] getSwerveModules(){
  return swerveModules;
}

@Override
public void periodic(){
}

}