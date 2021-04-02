package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DrivetrainSubsystem;

public class OI {
    public static OI instance;
    // private Joystick leftJoystick = new Joystick(0);
    // private Joystick rightJoystick = new Joystick(1);

    private Joystick driverXboxController = new Joystick(0); //this is no longer just a random number

    public OI() {
        // new JoystickButton(leftJoystick, 7).whenPressed(
        //     new InstantCommand(() -> DrivetrainSubsystem.getInstance().resetGyroscope())
        // );
    }

    public static OI getInstance(){
        if (instance == null){
          instance = new OI();
        }
        return instance;
      }

    public double getForward(){
        if(Math.abs(driverXboxController.getRawAxis(1)) > 0.1){
            return driverXboxController.getRawAxis(1); 
        }
        else{
            return 0.0;
        }
    }
    public double getStrafe(){
        if(Math.abs(driverXboxController.getRawAxis(0)) > 0.1){
            return driverXboxController.getRawAxis(0);
        }
        else{
            return 0.0;
        }
    }
    public double getRotation(){
        if(Math.abs(driverXboxController.getRawAxis(4)) > 0.1){
            return driverXboxController.getRawAxis(4);
        }
        else{
            return 0.0;
        }
    }
}
