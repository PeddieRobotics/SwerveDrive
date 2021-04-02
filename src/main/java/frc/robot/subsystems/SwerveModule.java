package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class SwerveModule {
    
    private final CANSparkMax driveMotor, angleMotor;
    
    private double driveSetpoint, angleSetpoint;

    private SwerveModuleState state;

    private final CANEncoder m_driveEncoder, m_turningEncoder;

    public SwerveModule(int driveMotorCanId, int angleMotorCanId){
        driveMotor = new CANSparkMax(driveMotorCanId, MotorType.kBrushless);
        angleMotor = new CANSparkMax(angleMotorCanId, MotorType.kBrushless);
        driveMotor.getPIDController().setP(0.00018);
        angleMotor.getPIDController().setP(0.005);
            
        m_driveEncoder = driveMotor.getEncoder();
        m_driveEncoder.setVelocityConversionFactor(1/(Constants.DRIVE_GEAR_RATIO *Constants.METERS_PER_SEC_TO_RPM));
        m_turningEncoder = angleMotor.getEncoder();
        m_turningEncoder.setPositionConversionFactor((2.0*Math.PI)/Constants.ANGLE_GEAR_RATIO);
    }

    public void setDriveMotor(double setpoint){
        driveMotor.set(setpoint);
    }

    public void setAngleMotor(double setpoint){
        angleMotor.set(setpoint);
    }
    
    public SwerveModuleState getCurrentState(){
        return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(normalizeAngle(m_turningEncoder.getPosition())));
    }

    public SwerveModuleState getDesiredState(){
        return state;
    }

    public double getDriveMotorSetpoint(){
        return driveSetpoint;
    }


    public double getAngleMotorSetpoint(){
        return angleSetpoint;
    }

    public void setDesiredState(SwerveModuleState desiredState){
      state = SwerveModuleState.optimize(desiredState, getCurrentState().angle);
      driveSetpoint = state.speedMetersPerSecond * Constants.METERS_PER_SEC_TO_RPM * Constants.DRIVE_GEAR_RATIO; // This is in rpm
      driveMotor.getPIDController().setReference(driveSetpoint, ControlType.kVelocity); 
      angleSetpoint = (state.angle.getRadians() / (2.0 * Math.PI)) * Constants.ANGLE_GEAR_RATIO;  // This is in # of rotations
      angleMotor.getPIDController().setReference(angleSetpoint, ControlType.kPosition);
    }

    public double normalizeAngle(double rad){
        double angle = rad % (2.0 * Math.PI);
        if (angle > Math.PI){
            angle = -(angle - Math.PI);
        }
        if (angle < -Math.PI){
            angle = -(angle + Math.PI);
        }
        return angle;
    }
}
