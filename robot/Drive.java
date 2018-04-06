package org.usfirst.frc.team5817.robot;
//import org.usfirst.frc.team5817.subsystems.Drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Servo;

public class Drive {
	
	private static Drive instance_;
	
	public static Drive getInstance() {
		if(instance_ == null) {
			instance_ = new Drive();
		}
		return instance_;
	}
	
	public static TalonSRX rightDriveOne;
	public static TalonSRX rightDriveTwo;
	public static TalonSRX rightDriveThree;
	public static TalonSRX leftDriveOne;
	public static TalonSRX leftDriveTwo;
	public static TalonSRX leftDriveThree;
	
	public static Servo shifterServoOne;
	public static Servo shifterServoTwo;
	
	public int kTimeoutMs = 10;
	
	private Drive(){
		rightDriveOne = new TalonSRX(0);
		rightDriveTwo = new TalonSRX(1);
		rightDriveThree = new TalonSRX(2);
		leftDriveOne = new TalonSRX(3);
		leftDriveTwo = new TalonSRX(4);
		leftDriveThree = new TalonSRX(5);
		
		shifterServoOne = new Servo(0);
		shifterServoTwo = new Servo(1);
		
	}
	
	public void motionMagic(){
		/* first choose the sensor */
		rightDriveOne.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, kTimeoutMs);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		rightDriveOne.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		rightDriveOne.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

		/* set the peak and nominal outputs */
		rightDriveOne.configNominalOutputForward(0, kTimeoutMs);
		rightDriveOne.configNominalOutputReverse(0, kTimeoutMs);
		rightDriveOne.configPeakOutputForward(1, kTimeoutMs);
		rightDriveOne.configPeakOutputReverse(-1, kTimeoutMs);

		/* set closed loop gains in slot0 - see documentation */
		rightDriveOne.selectProfileSlot(0, 0);
		rightDriveOne.config_kF(0, 0.5, kTimeoutMs);
		rightDriveOne.config_kP(0, 0.01, kTimeoutMs);
		rightDriveOne.config_kI(0, 0.0, kTimeoutMs);
		rightDriveOne.config_kD(0, 0.0, kTimeoutMs);
		/* set acceleration and vcruise velocity - see documentation */
		rightDriveOne.configMotionCruiseVelocity(2500, kTimeoutMs);
		rightDriveOne.configMotionAcceleration(2500, kTimeoutMs);
		/* zero the sensor */
		rightDriveOne.setSelectedSensorPosition(0, 0, kTimeoutMs);
		
		

		/* first choose the sensor */
		leftDriveOne.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, kTimeoutMs);
		leftDriveOne.setSensorPhase(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		leftDriveOne.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		leftDriveOne.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

		/* set the peak and nominal outputs */
		leftDriveOne.configNominalOutputForward(0, kTimeoutMs);
		leftDriveOne.configNominalOutputReverse(0, kTimeoutMs);
		leftDriveOne.configPeakOutputForward(1, kTimeoutMs);
		leftDriveOne.configPeakOutputReverse(-1, kTimeoutMs);

		/* set closed loop gains in slot0 - see documentation */
		leftDriveOne.selectProfileSlot(0, 0);
		leftDriveOne.config_kF(0, 0.5, kTimeoutMs);
		leftDriveOne.config_kP(0, 0.01, kTimeoutMs);
		leftDriveOne.config_kI(0, 0.0, kTimeoutMs);
		leftDriveOne.config_kD(0, 0, kTimeoutMs);
		/* set acceleration and vcruise velocity - see documentation */
		leftDriveOne.configMotionCruiseVelocity(2500, kTimeoutMs);
		leftDriveOne.configMotionAcceleration(2500, kTimeoutMs);
		/* zero the sensor */
		leftDriveOne.setSelectedSensorPosition(0, 0, kTimeoutMs);
			
			
		
		
	}
	
	public void rightSideControl(double value){
		rightDriveOne.set(ControlMode.PercentOutput, value);
		rightDriveTwo.set(ControlMode.PercentOutput, value);
		rightDriveThree.set(ControlMode.PercentOutput, value);
	}
	
	public void leftSideControl(double value){
		leftDriveOne.set(ControlMode.PercentOutput, value);
		leftDriveTwo.set(ControlMode.PercentOutput, value);
		leftDriveThree.set(ControlMode.PercentOutput, value);
	}
	
	public void setRightSidePosition(int value){
		rightDriveTwo.follow(rightDriveOne);
		rightDriveOne.set(ControlMode.MotionMagic, value);
	}
	
	public void setLeftSidePosition(int value){
		leftDriveTwo.follow(leftDriveOne);
		leftDriveOne.set(ControlMode.MotionMagic, value);
	}
	
	public void shiftUp(){
		shifterServoOne.set(-1.0);
		shifterServoTwo.set(-1.0);
	} 
	
	public void shiftDown(){
		shifterServoOne.set(1.0);
		shifterServoTwo.set(1.0);
	}
	
	public static void noMotion(){
		shifterServoOne.set(0.125);
		shifterServoTwo.set(0.125);
	}
	
	public void zeroSensors(){
		rightDriveOne.setSelectedSensorPosition(0, 0, 10);
		leftDriveOne.setSelectedSensorPosition(0, 0, 10);
	}
	
	public int getRightDriveVelocity(){
		return rightDriveOne.getSelectedSensorVelocity(0);
	}
	
	public int getLeftDriveVelocity(){
		return leftDriveOne.getSelectedSensorVelocity(0);
	}
	
	public int getRightDrivePosition(){
		return rightDriveOne.getSelectedSensorPosition(0);
	}
	
	public int getLeftDrivePosition(){
		return leftDriveOne.getSelectedSensorPosition(0);
	}
	
}
