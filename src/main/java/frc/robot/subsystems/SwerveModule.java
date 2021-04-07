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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class SwerveModule {
    
    private final CANSparkMax driveMotor, angleMotor;
    
    private SwerveModuleState state;

    private final CANEncoder m_driveEncoder, m_turningEncoder;

    private final ProfiledPIDController anglePIDController;

    private double desiredAngle;

    public SwerveModule(int driveMotorCanId, int angleMotorCanId){

        driveMotor = new CANSparkMax(driveMotorCanId, MotorType.kBrushless);
        angleMotor = new CANSparkMax(angleMotorCanId, MotorType.kBrushless);
        // left drive pid
        if(driveMotorCanId == 10 || driveMotorCanId == 4){
            driveMotor.getPIDController().setP(0.5);
        }
        // left angle pid
        if(angleMotorCanId == 1 || angleMotorCanId == 5){
            angleMotor.getPIDController().setP(0.5);
        }
        // right drive pid
        if(driveMotorCanId == 2 || driveMotorCanId == 6){
            driveMotor.getPIDController().setP(0.5);
        }
        // right angle pid
        if(angleMotorCanId == 3 || angleMotorCanId == 7){
            angleMotor.getPIDController().setP(0.5);
        }
    
        m_driveEncoder = driveMotor.getEncoder();
        m_driveEncoder.setVelocityConversionFactor(1/(Constants.DRIVE_GEAR_RATIO *Constants.METERS_PER_SEC_TO_RPM));
        m_turningEncoder = angleMotor.getEncoder();
        m_turningEncoder.setPositionConversionFactor((2.0*Math.PI)/Constants.ANGLE_GEAR_RATIO);
        anglePIDController = new ProfiledPIDController(0.15, 0, 0, new TrapezoidProfile.Constraints(2*Math.PI, 2*Math.PI));
        anglePIDController.enableContinuousInput(-Math.PI, Math.PI);
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

    public void setDesiredState(SwerveModuleState desiredState){
      state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getPosition()));
      driveMotor.getPIDController().setReference(state.speedMetersPerSecond, ControlType.kVelocity); 
      desiredAngle = anglePIDController.calculate(m_turningEncoder.getPosition(), state.angle.getRadians());
      angleMotor.set(desiredAngle);
      //angleMotor.getPIDController().setReference(state.angle.getRadians(), ControlType.kPosition);
    }

    public double normalizeAngle(double rad){
        double angle = rad % (2.0 * Math.PI);
        if (angle > Math.PI){
            angle = angle - 2.0*Math.PI;
        }
        else if (angle < -Math.PI){
            angle = angle + 2.0*Math.PI;
        }
        return angle;
    }

    public void setPIDFDrive(double p, double i, double d, double ff){
        driveMotor.getPIDController().setP(p);
        driveMotor.getPIDController().setI(i);
        driveMotor.getPIDController().setD(d);
        driveMotor.getPIDController().setFF(ff);
        
    }

    public void setPIDFAngle(double p, double i, double d, double ff){
        angleMotor.getPIDController().setP(p);
        angleMotor.getPIDController().setI(i);
        angleMotor.getPIDController().setD(d);
        angleMotor.getPIDController().setFF(ff);
        
    }

    public double getDesiredAngle(){
        return desiredAngle;
    }
}
