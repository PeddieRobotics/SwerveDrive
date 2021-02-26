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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SwerveDriveCommand;                                                                                                                                                                                              //Bao was here                  



public class DrivetrainSubsystem extends SubsystemBase {

  private static DrivetrainSubsystem instance;
  private ADIS16470_IMU gyroscope = new ADIS16470_IMU();

  /** Robot Dimensions **/
  // private static final double TRACKWIDTH = 19.0 //
  // private static final double WHEELBASE = 23.5;  //NEED TO CHANGE THIS FOR OUR ROBOT

  /**Angle Offsets */
  private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(0.0); //external magnet encoder - internal motor encoder
  private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(0.0); //external magnet encoder - internal motor encoder
  private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(0.0); //external magnet encoder - internal motor encoder
  private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(0.0); //external magnet encoder - internal motor encoder

  /**Time to build the Swerve Modules!! 
   * All the following port numbers are currently meaningless and random
  */
  
  private final CANSparkMax frontLeftDriveMotor  = new CANSparkMax(10, MotorType.kBrushless);
  private final CANSparkMax frontLeftAngleMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax frontRightDriveMotor = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax frontRightAngleMotor  = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax backLeftDriveMotor = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax backLeftAngleMotor  = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax backRightDriveMotor = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax backRightAngleMotor = new CANSparkMax(7, MotorType.kBrushless);

  private final CANSparkMax[] swerveMotors = {frontLeftDriveMotor,frontLeftAngleMotor,frontRightDriveMotor,frontRightAngleMotor,backLeftDriveMotor,backLeftAngleMotor,backRightDriveMotor,backRightAngleMotor};

  private final AnalogInput frontLeftAngleEncoder = new AnalogInput(1); //random numbers
  private final AnalogInput frontRightAngleEncoder = new AnalogInput(3); //random numbers
  private final AnalogInput backLeftAngleEncoder = new AnalogInput(5); //random numbers
  private final AnalogInput backRightAngleEncoder = new AnalogInput(7); //random numbers
  
  private final CANEncoder frontLeftDriveCANEncoder = frontLeftDriveMotor.getEncoder();  
  private final CANEncoder frontRightDriveCANEncoder = frontRightDriveMotor.getEncoder(); 
  private final CANEncoder backLeftDriveCANEncoder = backLeftDriveMotor.getEncoder();
  private final CANEncoder backRightDriveCANEncoder = backRightDriveMotor.getEncoder(); 
  private final CANEncoder frontLeftAngleCANEncoder = frontLeftAngleMotor.getEncoder();  
  private final CANEncoder frontRightAngleCANEncoder = frontLeftAngleMotor.getEncoder(); 
  private final CANEncoder backLeftAngleCANEncoder = frontLeftAngleMotor.getEncoder();
  private final CANEncoder backRightAngleCANEncoder = frontLeftAngleMotor.getEncoder(); 

  private final PIDController frontLeftDriveController = new PIDController(0.18, 0, 0); //random numbers
  private final PIDController frontRightDriveController = new PIDController(0.18, 0, 0); //random numbers
  private final PIDController backLeftDriveController = new PIDController(0.18, 0, 0); //random numbers
  private final PIDController backRightDriveController = new PIDController(0.18, 0, 0); //random numbers

  private final PIDController frontLeftAngleController = new PIDController(0.18, 0, 0); //random numbers
  private final PIDController frontRightAngleController = new PIDController(0.18, 0, 0); //random numbers
  private final PIDController backLeftAngleController = new PIDController(0.18, 0, 0); //random numbers
  private final PIDController backRightAngleController = new PIDController(0.18, 0, 0); //random numbers



  /** Creation of Swerve modules relative to the robot center */
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(Constants.TRACKWIDTH_M/2.0, Constants.WHEELBASE_M/2.0),
    new Translation2d(Constants.TRACKWIDTH_M/2.0, -Constants.WHEELBASE_M/2.0),
    new Translation2d(-Constants.TRACKWIDTH_M/2.0, Constants.WHEELBASE_M/2.0),
    new Translation2d(-Constants.TRACKWIDTH_M/2.0, -Constants.WHEELBASE_M/2.0)
  );

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    
  }

  public static DrivetrainSubsystem getInstance(){
    if (instance == null){
      instance = new DrivetrainSubsystem();
    }
    return instance;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  private double setTargetVelocity() {
    return (0.0);
  }
  
  private double setTargetAngle() {
    return (0.0);
  }
  
  public void drive(Translation2d translation, double  rotation, boolean fieldOriented) {
    rotation *= 2.0 / Math.hypot(Constants.WHEELBASE_M, Constants.TRACKWIDTH_M);
    ChassisSpeeds speeds;
    if(fieldOriented) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, Rotation2d.fromDegrees(gyroscope.getAngle()));
    } else {
      speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }

      SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds); //makes a SwerveModuleState array out of the ChassisSpeeds
       // Front left module state
      SwerveModuleState frontLeft = states[0];

      // Front right module state
      SwerveModuleState frontRight = states[1];

      // Back left module state
      SwerveModuleState backLeft = states[2];

      // Back right module state
      SwerveModuleState backRight = states[3];
     
      double frontLeftMotorSetpoint = frontLeft.speedMetersPerSecond * Constants.METERS_PER_SEC_TO_RPM * Constants.GEAR_RATIO; // This is in rpm
      frontLeftDriveMotor.getPIDController().setReference(frontLeftMotorSetpoint, ControlType.kVelocity); 
      double frontLeftAngleSetpoint = frontLeft.angle.getRadians() / (2 * Math.PI);  // This is in # of rotations
      frontLeftAngleMotor.getPIDController().setReference(frontLeftAngleSetpoint, ControlType.kPosition);

      double frontRightMotorSetpoint = frontRight.speedMetersPerSecond * Constants.METERS_PER_SEC_TO_RPM * Constants.GEAR_RATIO; // This is in rpm
      frontRightDriveMotor.getPIDController().setReference(frontRightMotorSetpoint, ControlType.kVelocity);
      double frontRightAngleSetpoint = frontRight.angle.getRadians() / (2 * Math.PI);  // This is in # of rotations
      frontRightAngleMotor.getPIDController().setReference(frontRightAngleSetpoint, ControlType.kPosition);
      
      double backLeftMotorSetpoint = backLeft.speedMetersPerSecond * Constants.METERS_PER_SEC_TO_RPM * Constants.GEAR_RATIO; // This is in rpm
      backLeftDriveMotor.getPIDController().setReference(backLeftMotorSetpoint, ControlType.kVelocity);
      double backLeftAngleSetpoint = backLeft.angle.getRadians() / (2 * Math.PI);  // This is in # of rotations
      backLeftAngleMotor.getPIDController().setReference(backLeftAngleSetpoint, ControlType.kPosition);

      double backRightMotorSetpoint = backRight.speedMetersPerSecond * Constants.METERS_PER_SEC_TO_RPM * Constants.GEAR_RATIO;// This is in rpm
      backRightDriveMotor.getPIDController().setReference(backRightMotorSetpoint, ControlType.kVelocity);
      double backRightAngleSetpoint = backRight.angle.getRadians() / (2 * Math.PI); // This is in # of rotations
      backRightAngleMotor.getPIDController().setReference(backRightAngleSetpoint, ControlType.kPosition);
  }

public void resetGyroscope() {
	gyroscope.reset();
}

public void setMotor(int motorID, double setpoint){

  if(setpoint <= 0.3){
    swerveMotors[motorID % 10].set(setpoint);
  }
  
  /*
  switch (motorID){
    case 10:
      frontLeftDriveMotor.set(setpoint);
      break;
    case 1:
      frontLeftAngleMotor.set(setpoint);
      break;
    case 2:
      frontRightDriveMotor.set(setpoint);
      break;
    case 3:
      frontRightAngleMotor.set(setpoint);
      break;
    case 4:
      backLeftDriveMotor.set(setpoint);
      break;
    case 5:
      backLeftAngleMotor.set(setpoint);
      break;
    case 6:
      backRightDriveMotor.set(setpoint);
      break;
    case 7:
      backRightAngleMotor.set(setpoint);
      break;
    default:
      break;
  }*/
}

}