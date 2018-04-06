package org.usfirst.frc.team5817.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Controller {
	
private static Controller instance_ = new Controller();
	
	public static Controller getInstance() {
		return instance_;
	}
	public Joystick driverController;
	public Joystick codriverController;
	
	boolean lastValue;
	boolean lastValueTwo;
	
	private Controller(){
		driverController = new Joystick(0);
		//created a new object of type Joystick which is indexed at zero.
		
		codriverController = new Joystick(1);
		//created a new object of typer Joystick indexed at zero.
	}
	
	
	public double getXLeft(){
		return driverController.getRawAxis(0);
		//returns the value of the X axis in the left joystick(-1 to 1)
	}
	public double getYLeft(){
		return driverController.getRawAxis(1);
		//returns the value of the Y axis on the left joystick(-1 to 1)
	}
	public double getRightTrigger(){
		return driverController.getRawAxis(3);
		//returns the depth to which the trigger is pressed
		//right trigger returns 0 to 1
	}
	public double getLeftTrigger(){
		return driverController.getRawAxis(2);
		//returns the depth to which the trigger is pressed
		//left trigger returns 0 to 1
	}
	public double getXRight(){
		return driverController.getRawAxis(4);
		//returns the value of the X axis in the right joystick(-1 to 1)
	}
	public double getYRight(){
		return driverController.getRawAxis(5);
		//returns the value of the Y axis on the right joystick(-1 to 1)
	}
	public int getDpad(){
		return driverController.getPOV();
		//returns the value of the Dpad in degrees starting at 0(straight up)
		//up = 0
		//up and left = 45
		//left = 90
		//left and down = 135
		//down = 180
		//down and right = 225
		//right = 270
		//right and up = 315
	}
	public boolean getButtonA(){
		return driverController.getRawButton(1);
		//returns true when the B button is pressed otherwise false
	}
	public boolean getButtonB(){
		return driverController.getRawButton(2);
		//returns true when the A button is pressed otherwise false
	}
	public boolean getButtonX(){
		return driverController.getRawButton(3);
		//returns true when the X button is pressed otherwise false
	}
	public boolean getButtonY(){
		return driverController.getRawButton(4);
		//returns true when the Y button is pressed otherwise false
	}
	public boolean getLeftBumper(){
		return driverController.getRawButton(5);
		//returns true when the Left Bumper is pressed otherwise false
	}
	public boolean getRightBumper(){
		return driverController.getRawButton(6);
		//returns true when the Right Bumper is pressed otherwise false
	}
	public boolean getBackButton(){
		return driverController.getRawButton(7);
		//returns true when the Back button is pressed otherwise false
	}
	public boolean getStartButton(){
		return driverController.getRawButton(8);
		//returns true when the start button is pressed otherwise false
	}
	public boolean getLeftJoystickPress(){
		return driverController.getRawButton(9);
		//returns true when the left Joystick is pressed down otherwise false
	}
	public boolean getRightJoystickPress()
	{
		return driverController.getRawButton(10);
		//returns true when the right Joystick is pressed down otherwise false
	}
	public double getXLeft2(){
		return codriverController.getRawAxis(0);
		//returns the value of the X axis in the left joystick(-1 to 1)
	}
	public double getYLeft2(){
		return codriverController.getRawAxis(1);
		//returns the value of the Y axis on the left joystick(-1 to 1)
	}
	public double getRightTrigger2(){
		return codriverController.getRawAxis(3);
		//returns the depth to which the trigger is pressed
		//right trigger returns 0 to 1
	}
	public double getLeftTrigger2(){
		return codriverController.getRawAxis(2);
		//returns the depth to which the trigger is pressed
		//left trigger returns 0 to 1
	}
	public double getXRight2(){
		return codriverController.getRawAxis(4);
		//returns the value of the X axis in the right joystick(-1 to 1)
	}
	public double getYRight2(){
		return codriverController.getRawAxis(5);
		//returns the value of the Y axis on the right joystick(-1 to 1)
	}
	public int getDpad2(){
		return codriverController.getPOV();
		//returns the value of the Dpad in degrees starting at 0(straight up)
		//up = 0
		//up and left = 45
		//left = 90
		//left and down = 135
		//down = 180
		//down and right = 225
		//right = 270
		//right and up = 315
	}
	public boolean getButtonA2(){
		return codriverController.getRawButton(1);
		//returns true when the B button is pressed otherwise false
	}
	public boolean getButtonB2(){
		return codriverController.getRawButton(2);
		//returns true when the A button is pressed otherwise false
	}
	public boolean getButtonX2(){
		return codriverController.getRawButton(3);
		//returns true when the X button is pressed otherwise false
	}
	public boolean getButtonY2(){
		return codriverController.getRawButton(4);
		//returns true when the Y button is pressed otherwise false
	}
	public boolean getLeftBumper2(){
		return codriverController.getRawButton(5);
		//returns true when the Left Bumper is pressed otherwise false
	}
	public boolean getRightBumper2(){
		return codriverController.getRawButton(6);
		//returns true when the Right Bumper is pressed otherwise false
	}
	public boolean getBackButton2(){
		return codriverController.getRawButton(7);
		//returns true when the Back button is pressed otherwise false
	}
	public boolean getStartButton2(){
		return codriverController.getRawButton(8);
		//returns true when the start button is pressed otherwise false
	}
	public boolean getLeftJoystickPress2(){
		return codriverController.getRawButton(9);
		//returns true when the left Joystick is pressed down otherwise false
	}
	public boolean getRightJoystickPress2() {
		return codriverController.getRawButton(10);
		//returns true when the right Joystick is pressed down otherwise false
	}

	boolean wasDpadUpPressedLast = false;
	
	public boolean onDpadUpPressed() {
		if(!wasDpadUpPressedLast && (driverController.getPOV() == 0 || codriverController.getPOV() == 0)) {
			wasDpadUpPressedLast = true;
			return true;
		}
		wasDpadUpPressedLast = (driverController.getPOV() == 0 || codriverController.getPOV() == 0);
		return false;
	}
	
	boolean wasDpadDownPressedLast = false;
	public boolean onDpadDownPressed() {
		if(!wasDpadDownPressedLast && (driverController.getPOV() == 180 || codriverController.getPOV() == 180)) {
			wasDpadDownPressedLast = true;
			return true;
		}
		wasDpadDownPressedLast = (driverController.getPOV() == 180 || codriverController.getPOV() == 180);
		return false;
	}
	
}	