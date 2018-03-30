package org.usfirst.frc.team5817.robot;

import org.usfirst.frc.team5817.robot.auto.AutoMode;
import org.usfirst.frc.team5817.robot.auto.OneSwitchOppositeSide;
import org.usfirst.frc.team5817.robot.auto.OneSwitchSameSide;
import org.usfirst.frc.team5817.robot.auto.OneScaleTwoSwitchSameSide;
import org.usfirst.frc.team5817.robot.auto.AutoSelector;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	
	private final Drive drive_ = Drive.getInstance();
	private final Controller driverController_ = Controller.getInstance();
	private final Wrist wrist_ = Wrist.getInstance();
	private final Arm arm_ = Arm.getInstance();
	private final NavxHelper navx_ = NavxHelper.getInstance();
	
	private Position position = Position.MANUAL;
	
	double leftDeadZoneXY = 0.2;								// Dead Zone Controls
	double leftNegativeDeadZoneXY = -0.2;						// how much you have to move 
	double rightDeadZoneXY = 0.2;								// the Joystick before the robot
	double rightNegativeDeadZoneXY = -0.2;						// moves
	
	double rightSensitivityX = 0.4;							// Sensitivity Controls 
	double rightSensitivityY = 0.4;							// 1 = High Sensitivity
	double leftSensitivityX = 0.6;								// 0 = Low Sensitivity
	double leftSensitivityY = 0.6;
	
	
	final AutoMode oneScaleTwoSwitchSameSide = new OneScaleTwoSwitchSameSide();
	final AutoMode oneSwitchOppositeSide = new OneSwitchOppositeSide();
	final AutoMode oneSwitchSameSide = new OneSwitchSameSide();
	final AutoMode autoSelector = new AutoSelector();
	AutoMode autoSelected;
	SendableChooser<AutoMode> chooser = new SendableChooser<>();

	@Override
	public void robotInit() {
		chooser.addDefault("One Scale Two Switch Same Side", oneScaleTwoSwitchSameSide);
		chooser.addObject("One Switch Opposite Side", oneSwitchOppositeSide);
		chooser.addObject("One Switch Same Side", oneSwitchSameSide);
		chooser.addObject("Auto Choosing", autoSelector);
		
		SmartDashboard.putData("Auto choices", chooser);
		
		wrist_.motionMagic();
		arm_.motionMagic();
		drive_.motionMagic();
	}


		//armMotorOne.selectProfileSlot(0, 0);
		

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		autoSelected.initialize();
	}
	
	public void disabledPeriodic(){
		if (driverController_.getStartButton2()){
			arm_.zero();
			wrist_.zero();
		}
		
		if (driverController_.getStartButton()){
			drive_.zeroSensors();
		} else if(driverController_.getBackButton()) {
			navx_.zeroAngle();
		}
		
		SmartDashboard.putNumber("Arm Position", arm_.getArmPosition());
		SmartDashboard.putNumber("Wrist Position", wrist_.getWristPosition());
		SmartDashboard.putNumber("Right Drive Velocity", drive_.getRightDriveVelocity());
		SmartDashboard.putNumber("Left Drive Velocity", drive_.getLeftDriveVelocity());
		SmartDashboard.putNumber("Right Drive Position", drive_.getRightDrivePosition());
		SmartDashboard.putNumber("Left Drive Position", drive_.getLeftDrivePosition());
		SmartDashboard.putNumber("Gyro Angle", navx_.getAngle());
		SmartDashboard.putBoolean("Gyro Connection", navx_.isConnected());
		SmartDashboard.putNumber("Max Arm Current", arm_.getCurrent());
		SmartDashboard.putNumber("Max Wrist Current", wrist_.getCurrent());
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		autoSelected.execute();
		SmartDashboard.putNumber("Gyro Angle", navx_.getAngle());
		SmartDashboard.putNumber("Max Arm Current", arm_.getCurrent());
		SmartDashboard.putNumber("Max Wrist Current", wrist_.getCurrent());
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		SmartDashboard.putNumber("Arm Position", arm_.getArmPosition());
		SmartDashboard.putNumber("Wrist Position", wrist_.getWristPosition());
		SmartDashboard.putNumber("Right Drive Velocity", drive_.getRightDriveVelocity());
		SmartDashboard.putNumber("Left Drive Velocity", drive_.getLeftDriveVelocity());
		SmartDashboard.putNumber("Right Drive Position", drive_.getRightDrivePosition());
		SmartDashboard.putNumber("Left Drive Position", drive_.getLeftDrivePosition());
		SmartDashboard.putNumber("Gyro Angle", navx_.getAngle());
		SmartDashboard.putBoolean("Gyro Connection", navx_.isConnected());
		SmartDashboard.putNumber("Max Arm Current", arm_.getCurrent());
		SmartDashboard.putNumber("Max Wrist Current", wrist_.getCurrent());

		
		if (driverController_.getStartButton()){
			drive_.zeroSensors();
		}
		
		// Preset arm and wrist positions for scoring on different targets
		if (driverController_.getButtonA2()) {
			// Preset for picking up a cube off the ground		
			position = Position.GROUND;
		}else if(driverController_.getRightBumper2()){
			position = Position.EXCHANGE;
		}else if (driverController_.getButtonB2()){
			// Preset for scoring in the switch	
			position = Position.ZERO;
		}else if (driverController_.getButtonX2()){
			// Preset for scoring on the Scale
			position = Position.SWITCH;
		}else if (driverController_.getButtonY2()){
			// Preset for scoring on the scale backwards
			position = Position.SCALEBACK;
		}else if(driverController_.getYRight2() > rightDeadZoneXY || driverController_.getYRight2() < rightNegativeDeadZoneXY){
			position = Position.MANUAL;
		}else if(driverController_.getYLeft2() > rightDeadZoneXY || driverController_.getYLeft2() < rightNegativeDeadZoneXY){
			position = Position.MANUAL;
		}else if(driverController_.getDpad2() == 0 || driverController_.getDpad() == 0){
			position = Position.SCALE;
		}else if(driverController_.getDpad2() == 180 || driverController_.getDpad() == 180){
			position = Position.LOW_PROFILE_SCALE;
		}
		else if (driverController_.getButtonA()) {
			// Preset for picking up a cube off the ground		
			position = Position.GROUND;
		}else if(driverController_.getRightBumper()){
			position = Position.EXCHANGE;
		}else if (driverController_.getButtonB()){
			// Preset for scoring in the switch	
			position = Position.ZERO;
		}else if (driverController_.getButtonX()){
			// Preset for scoring on the Scale
			position = Position.SWITCH;
		}else if (driverController_.getButtonY()){
			// Preset for scoring on the scale backwards
			position = Position.SCALEBACK;
		}else if(driverController_.getLeftBumper2()){
			position = Position.CLIMB;
		}
		
		switch(position){
		case ZERO:
			arm_.setArmPosition(10);
			wrist_.setWristPosition(10);
			break;
		case GROUND:
			if(arm_.getArmPosition() > 1000 && wrist_.getWristPosition() > 3000) {
				wrist_.setWristPosition(50);
			} else if(arm_.getArmPosition() > 1000) {
				arm_.setArmPosition(100);
			} else {
				arm_.setArmPosition(100);
				wrist_.setWristPosition(4000);
			}
			break;
		case EXCHANGE:
			if(arm_.getArmPosition() > 1000 && wrist_.getWristPosition() > 3000) {
				wrist_.setWristPosition(50);
			} else if(arm_.getArmPosition() > 1000) {
				arm_.setArmPosition(100);
			} else {
				arm_.setArmPosition(100);
				wrist_.setWristPosition(3800);
			}
			break;
		case SWITCH:
			arm_.setArmPosition(100);
			wrist_.setWristPosition(1500);
			break;
		case SCALE:
			//6600 Arm
			//9750 Wrist
			if(wrist_.getWristPosition() > 3000 && arm_.getArmPosition() < 1500){
				wrist_.setWristPosition(50);
				arm_.setArmPosition(300);
			}else if(arm_.getArmPosition() < 3000){
				arm_.setArmPosition(6600);
				wrist_.setWristPosition(50);
			}else{
				arm_.setArmPosition(6600);
				wrist_.setWristPosition(9000);
			}
			break;
		case SCALEBACK:
			if(wrist_.getWristPosition() > 3000 && arm_.getArmPosition() < 1500){
				wrist_.setWristPosition(50);
				arm_.setArmPosition(300);
			}else if(arm_.getArmPosition() < 1500){
				arm_.setArmPosition(5000);
				wrist_.setWristPosition(50);
			}else{
				arm_.setArmPosition(4400);
				wrist_.setWristPosition(1850);
			}
			break;
		case LOW_PROFILE_SCALE:
			if(wrist_.getWristPosition() > 3000 && arm_.getArmPosition() < 1000){
				wrist_.setWristPosition(50);
				arm_.setArmPosition(300);
			}else if(arm_.getArmPosition() < 1500){
				arm_.setArmPosition(5300);
				wrist_.setWristPosition(50);
			}else{
				arm_.setArmPosition(5300);
				wrist_.setWristPosition(2700);
			}
			break;
		case CLIMB:
			if(wrist_.getWristPosition() > 3000 && arm_.getArmPosition() < 1500){
				wrist_.setWristPosition(50);
				arm_.setArmPosition(300);
			}else if(arm_.getArmPosition() < 1500){
				arm_.setArmPosition(5000);
				wrist_.setWristPosition(50);
			}else if (arm_.getArmPosition() < 4300){
				arm_.setArmPosition(4400);
				wrist_.setWristPosition(1750);
			} else if (arm_.getArmPosition() < 7000){
				wrist_.setWristPosition(5000);
				arm_.setArmPosition(7000);
			}
			break;
		case MANUAL:
			// Manual Control of the wrist Movements
						if (driverController_.getYRight2() > rightDeadZoneXY){
							wrist_.manualWristControl(driverController_.getYRight2());
						}else if (driverController_.getYRight2() < rightNegativeDeadZoneXY){
							wrist_.manualWristControl(driverController_.getYRight2());
						}else{
							wrist_.manualWristControl(0.0);
						}
						
						// Manual Control of the arm movements
						if (driverController_.getYLeft2() > leftDeadZoneXY){	
							arm_.manualArmController(driverController_.getYLeft2());
						}else if (driverController_.getYLeft2() < leftNegativeDeadZoneXY){
							arm_.manualArmController(driverController_.getYLeft2());
						}else{
							arm_.manualArmController(0.0);;
						}
			break;
		default:
			// Manual Control of the wrist Movements
						if (driverController_.getYRight2() > rightDeadZoneXY){
							wrist_.manualWristControl(driverController_.getYRight2());
						}else if (driverController_.getYRight2() < rightNegativeDeadZoneXY){
							wrist_.manualWristControl(driverController_.getYRight2());
						}else{
							wrist_.manualWristControl(0.0);
						}
						
						// Manual Control of the arm movements
						if (driverController_.getYLeft2() > leftDeadZoneXY){	
							arm_.manualArmController(driverController_.getYLeft2());
						}else if (driverController_.getYLeft2() < leftNegativeDeadZoneXY){
							arm_.manualArmController(driverController_.getYLeft2());
						}else{
							arm_.manualArmController(0.0);;
						}
			break;
			
		}
	
		// Drive Code
		if (driverController_.getYLeft() > leftDeadZoneXY){
			drive_.rightSideControl((driverController_.getYLeft() + (driverController_.getXRight() * rightSensitivityX)) * -1);
			drive_.leftSideControl((driverController_.getYLeft() - (driverController_.getXRight() * rightSensitivityX)) * -1);
		}else if (driverController_.getYLeft() < leftNegativeDeadZoneXY){
			drive_.rightSideControl((driverController_.getYLeft() + (driverController_.getXRight() * rightSensitivityX)) * -1);
			drive_.leftSideControl((driverController_.getYLeft() - (driverController_.getXRight() * rightSensitivityX)) * -1);
		}else{
			drive_.leftSideControl(driverController_.getXRight());
			drive_.rightSideControl(driverController_.getXRight() * -1);
		}
		
		// zeroing the arm and wrist positions
		if (driverController_.getStartButton2()){
			arm_.zero();
			wrist_.zero();
		}else if(driverController_.getBackButton()) {
			navx_.zeroAngle();
		}
		
		// Opening and closing the Fingers(flippity floppitys) to grab a cube
		if (driverController_.getLeftTrigger2() > 0.1){
			wrist_.fingersOpen();
		} else if (driverController_.getRightTrigger2() > 0.1){
			wrist_.fingersClose();
		}else{
			wrist_.fingerStop();
		}
		
		// Running the Intake wheels to pick up or release a cube
		if (driverController_.getLeftTrigger() > 0.1){
			wrist_.intake();
		}else if(driverController_.getRightTrigger() > 0.1){
			double value = Math.pow(driverController_.getRightTrigger(), 3) * 1.0;
			value = (driverController_.getRightTrigger() < 0.7) ? 0.3 : value;
			wrist_.outtakeValue(value);
		}else{
			wrist_.stop();
		}
		
		if (driverController_.getRightBumper()){
			drive_.shiftDown();
		}else if (driverController_.getLeftBumper()){
			drive_.shiftUp();
		}
		
		if(driverController_.getBackButton2()){
			arm_.goingUP();
		}else{
			arm_.stopClimber();
		}
		
		
		
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		// :)
	}
}

