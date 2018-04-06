package org.usfirst.frc.team5817.robot;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.*;

public class Wrist {
	
	private static Wrist instance_;
	
	public static Wrist getInstance() {
		if(instance_ == null) {
			instance_ = new Wrist();
		}
	    return instance_;
	}
	
	public static TalonSRX wristMotion;
	public static TalonSRX intakeOne;
	public static TalonSRX intakeTwo;
	public static TalonSRX fingerOne;
	public static TalonSRX fingerTwo;
	
	private Wrist(){
		
		wristMotion = new TalonSRX(13);
		intakeOne = new TalonSRX(9);
		intakeTwo = new TalonSRX(10);
		fingerOne = new TalonSRX(11);
		fingerTwo = new TalonSRX(12);
		
	}

	private final double maxWristMotion = 8954;
	private final double minWristMotion = 0;
	
	public void motionMagic(){
		
		/* first choose the sensor */
		wristMotion.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		wristMotion.setSensorPhase(false);
		wristMotion.setInverted(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		wristMotion.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);
		wristMotion.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);

		/* set the peak and nominal outputs */
		wristMotion.configNominalOutputForward(0, 10);
		wristMotion.configNominalOutputReverse(0, 10);
		wristMotion.configPeakOutputForward(1, 10);
		wristMotion.configPeakOutputReverse(-1, 10);

		/* set closed loop gains in slot0 - see documentation */
		wristMotion.selectProfileSlot(0, 0);
		wristMotion.config_kF(0, 0.5, 10);
		wristMotion.config_kP(0, 3.5, 10);
		wristMotion.config_kI(0, 0.0, 10);
		wristMotion.config_kD(0, 100, 10);
		
		/* set acceleration and vcruise velocity - see documentation */
		wristMotion.configMotionCruiseVelocity(4000, 10);
		wristMotion.configMotionAcceleration(8000, 10);
		
		/* zero the sensor */
		wristMotion.setSelectedSensorPosition(0, 0, 10);
	}
	
	public void setWristPosition(int value){
		double adjustedValue = value * (5.0/8.0);
		wristMotion.set(ControlMode.MotionMagic, adjustedValue);
	}
	
	public void fingersOpen(){
		fingerOne.set(ControlMode.PercentOutput, 0.8);
		fingerTwo.set(ControlMode.PercentOutput, 0.8);
	}
	
	public void fingersClose(){
		fingerOne.set(ControlMode.PercentOutput, -0.8);
		fingerTwo.set(ControlMode.PercentOutput, -0.8);
	}
	
	public void fingerStop(){
		fingerOne.set(ControlMode.PercentOutput, 0.0);
		fingerTwo.set(ControlMode.PercentOutput, 0.0);
	}

	public void manualWristControl(double value){
		wristMotion.set(ControlMode.PercentOutput, value);
	}
	
	public void zero(){
		wristMotion.setSelectedSensorPosition(0, 0, 10);
	}
	public double getWristPosition(){
		return ((double) wristMotion.getSensorCollection().getQuadraturePosition()) * (8.0/5.0);
	}
	
	public void intake(){
		
		intakeOne.set(ControlMode.PercentOutput, 0.75);
		intakeTwo.set(ControlMode.PercentOutput, 0.75);
	}
	
	public void outtake(){
		intakeOne.set(ControlMode.PercentOutput, -0.6);
		intakeTwo.set(ControlMode.PercentOutput, -0.6);
	}
	
	public void outtakeValue(double value){
		intakeOne.set(ControlMode.PercentOutput, -value);
		intakeTwo.set(ControlMode.PercentOutput, -value);
	}
	
	public void outtakeSlow(){
		intakeOne.set(ControlMode.PercentOutput, -0.30);
		intakeTwo.set(ControlMode.PercentOutput, -0.30);
	}
	
	public void shoot(){
		intakeOne.set(ControlMode.PercentOutput, -1.0);
		intakeTwo.set(ControlMode.PercentOutput, -1.0);
	}
	
	public void stop(){
		intakeOne.set(ControlMode.PercentOutput, 0.0);
		intakeTwo.set(ControlMode.PercentOutput, 0.0);
	}
	public double getCurrent(){
		double maxCurrent = 0;
		if (wristMotion.getOutputCurrent() > maxCurrent){
			maxCurrent = wristMotion.getOutputCurrent();
		};
		return maxCurrent;
	}
}