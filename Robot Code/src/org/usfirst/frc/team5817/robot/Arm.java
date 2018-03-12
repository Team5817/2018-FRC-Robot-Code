package org.usfirst.frc.team5817.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class Arm {
	
private static Arm instance_ = new Arm();
	
	public static Arm getInstance() {
		return instance_;
	}
	
	public static TalonSRX armMotorOne;
	public static TalonSRX climber;
	
	private Arm(){
		
		armMotorOne = new TalonSRX(6);
		climber = new TalonSRX(7);
		
	}

	private static int kTimeoutMs = 10;
	
	public void motionMagic(){
		/* first choose the sensor */
		armMotorOne.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, kTimeoutMs);
		armMotorOne.setSensorPhase(false);
		armMotorOne.setInverted(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		armMotorOne.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		armMotorOne.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

		/* set the peak and nominal outputs */
		armMotorOne.configNominalOutputForward(0, kTimeoutMs);
		armMotorOne.configNominalOutputReverse(0, kTimeoutMs);
		armMotorOne.configPeakOutputForward(1, kTimeoutMs);
		armMotorOne.configPeakOutputReverse(-1, kTimeoutMs);

		/* set closed loop gains in slot0 - see documentation */
		armMotorOne.selectProfileSlot(0, 0);
		armMotorOne.config_kF(0, 0.5, kTimeoutMs);
		armMotorOne.config_kP(0, 1.25, kTimeoutMs);
		armMotorOne.config_kI(0, 0.0, kTimeoutMs);
		armMotorOne.config_kD(0, 15, kTimeoutMs);
		/* set acceleration and cruise velocity - see documentation */
		armMotorOne.configMotionCruiseVelocity(4000, kTimeoutMs);
		armMotorOne.configMotionAcceleration(9000, kTimeoutMs);
		/* zero the sensor */
		armMotorOne.setSelectedSensorPosition(0, 0, kTimeoutMs);
		
		
	}
	
	public void setArmPosition(int value){
		
		armMotorOne.set(ControlMode.MotionMagic, value);
	}
	
	public void manualArmController(double value){
		armMotorOne.set(ControlMode.PercentOutput, value * -1);
	}
	
	public void zero(){
		armMotorOne.setSelectedSensorPosition(0, 0, 10);
	}
	
	public double getArmPosition(){
		return armMotorOne.getSensorCollection().getQuadraturePosition();
	}
	
	public void goingUP(){
		climber.set(ControlMode.PercentOutput, 90.0);
	}
	
	public void stopClimber(){
		climber.set(ControlMode.PercentOutput, 0.0);
	}
	public double getCurrent(){
		double maxCurrent = 0;
		if (armMotorOne.getOutputCurrent() > maxCurrent){
			maxCurrent = armMotorOne.getOutputCurrent();
		};
		return maxCurrent;
	}
}
