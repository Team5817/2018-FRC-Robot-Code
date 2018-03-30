package org.usfirst.frc.team5817.robot.auto;

import org.usfirst.frc.team5817.robot.Arm;
import org.usfirst.frc.team5817.robot.Controller;
import org.usfirst.frc.team5817.robot.Drive;
import org.usfirst.frc.team5817.robot.NavxHelper;
import org.usfirst.frc.team5817.robot.Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TwoScaleSameSide extends AutoMode {
	
	private final Drive drive_ = Drive.getInstance();
	private final Controller driverController_ = Controller.getInstance();
	private final Wrist wrist_ = Wrist.getInstance();
	private final Arm arm_ = Arm.getInstance();
	private final NavxHelper navx_ = NavxHelper.getInstance();
	
	private State state = State.DRIVE_TO_SCALE;
	
	private Timer timer = new Timer();
	
	private enum State {
		DRIVE_TO_SCALE(4.0),
		TURN_TO_INTAKE_AND_LOWER_ARM(1.0),
		DRIVE_FORWARD_AND_INTAKE(1.5),
		DRIVE_BACKWARD_AND_RAISE_ARM(5.0),
		SPIT(2.0),
		END(0.0);
		
		double time;
		State(double time) {
			this.time = time;
		}
		
		double getTime() {
			return time;
		}
	}

	@Override
	public void initialize() {
		
		drive_.zeroSensors();
		wrist_.zero();
		arm_.zero();
		navx_.zeroAngle();
		
		timer.reset();
		timer.start();
		
		state = State.DRIVE_TO_SCALE;
	
	}
	
	@Override
	public void execute() {
		switch(state){
		
		case DRIVE_TO_SCALE:
			if(timer.get() <= state.getTime()) {
				if(timer.get() <= 0.5) {
					double gyro_correction = 0.02 * navx_.getAngle();
					drive_.leftSideControl((2 * timer.get() * -0.75) - gyro_correction);
					drive_.rightSideControl((2 * timer.get() * -0.75) + gyro_correction);
				} else if(Math.abs(drive_.getLeftDrivePosition()) < 270000) {
					double gyro_correction = 0.02 * navx_.getAngle();
					drive_.leftSideControl(-0.75 - gyro_correction);
					drive_.rightSideControl(-0.75 + gyro_correction);
				} else if(Math.abs(drive_.getLeftDrivePosition()) < 350000) {
					double left_distance_error = -420000 - drive_.getLeftDrivePosition();
					double right_distance_error = -400000 - drive_.getRightDrivePosition();
					
					double left_value = 0.0;
					double right_value = 0.0;
					
					if(Math.abs(left_distance_error) < 220000) {
						double gyro_correction = 0.01 * (18 - navx_.getAngle());
						left_value = -0.75 + gyro_correction;
						if(Math.abs(left_value) > 0.95) left_value = Math.signum(left_value) * 0.95;
						right_value = -0.75 - gyro_correction;
						if(Math.abs(right_value) > 0.95) right_value = Math.signum(left_value) * 0.95;
					} else {
						double gyro_correction = 0.01 * (navx_.getAngle());
						left_value = -0.75 - gyro_correction;
						if(Math.abs(left_value) > 0.85) left_value = Math.signum(left_value) * 0.85;
						right_value = -0.75 + gyro_correction;
						if(Math.abs(right_value) > 0.85) right_value = Math.signum(left_value) * 0.85;
					}
					
					drive_.leftSideControl(left_value);
					drive_.rightSideControl(right_value);
				} else {
					double left_distance_error = -420000 - drive_.getLeftDrivePosition();
					double right_distance_error = -400000 - drive_.getRightDrivePosition();
					double gyro_correction = 0.01 * (13 - navx_.getAngle());
					
					double left_value = (0.0002 * left_distance_error) + gyro_correction;
					if(Math.abs(left_value) > 0.25) left_value = Math.signum(left_value) * 0.25;
					double right_value = (0.0002 * right_distance_error) - gyro_correction;
					if(Math.abs(right_value) > 0.25) right_value = Math.signum(left_value) * 0.25;
					
					drive_.leftSideControl(left_value);
					drive_.rightSideControl(right_value);
					arm_.setArmPosition(3600);
					wrist_.setWristPosition(2000);
				}
				
				if(Math.abs(drive_.getLeftDrivePosition()) > 200000) {
					arm_.setArmPosition(3600);
				}
				if(Math.abs(drive_.getLeftDrivePosition()) > 220000) {
					wrist_.setWristPosition(2000);
				} else {
					wrist_.setWristPosition(0);
				}
				
				
				
				
				
				if(Math.abs(drive_.getLeftDrivePosition()) > 410000) {
					wrist_.outtakeValue(0.50);
				}
			} else {
				wrist_.stop();
				state = State.TURN_TO_INTAKE_AND_LOWER_ARM;
				timer.stop();
				timer.reset();
				timer.start();
			}
			break;
			
		case TURN_TO_INTAKE_AND_LOWER_ARM:
			if(timer.get() <= state.getTime()) {
				double gyro_correction = 0.02 * (-19 - navx_.getAngle());
				if(Math.abs(gyro_correction) > 0.50) gyro_correction = 0.50 * Math.signum(gyro_correction);
				drive_.leftSideControl(gyro_correction);
				drive_.rightSideControl(-gyro_correction);
				wrist_.fingersOpen();
				
				if(arm_.getArmPosition() > 3000) {
					wrist_.setWristPosition(50);
					arm_.setArmPosition(50);
				} else {
					arm_.setArmPosition(50);
					wrist_.setWristPosition(4350);
				}
			} else {
				state = State.DRIVE_FORWARD_AND_INTAKE;
				timer.stop();
				timer.reset();
				timer.start();
				drive_.zeroSensors();
			}
			break;
			
		case DRIVE_FORWARD_AND_INTAKE:
			if(timer.get() <= state.getTime()) {
				double left_distance_error = 95000 - drive_.getLeftDrivePosition();
				double right_distance_error = 95000 - drive_.getRightDrivePosition();
				double gyro_correction = 0.16 * (-19 - navx_.getAngle());
				
				double left_value = (0.0004 * left_distance_error) + gyro_correction;
				if(Math.abs(left_value) > 0.65) left_value = Math.signum(left_value) * 0.65;
				double right_value = (0.0004 * right_distance_error) - gyro_correction;
				if(Math.abs(right_value) > 0.65) right_value = Math.signum(left_value) * 0.65;
				
				wrist_.intake();
				if(Math.abs(left_distance_error) < 2500 && Math.abs(right_distance_error) < 2500) {
					wrist_.fingersClose();
					drive_.leftSideControl(0.0);
					drive_.rightSideControl(0.0);
				}else{
					drive_.leftSideControl(left_value);
					drive_.rightSideControl(right_value);
				}
			} else {
				drive_.zeroSensors();
				wrist_.stop();
				state = State.DRIVE_BACKWARD_AND_RAISE_ARM;
				timer.stop();
				timer.reset();
				timer.start();
			}
			break;
			
		case DRIVE_BACKWARD_AND_RAISE_ARM:
			if(timer.get() <= state.getTime()) {
				double left_distance_error = -90000 - drive_.getLeftDrivePosition();
				double right_distance_error = -90000 - drive_.getRightDrivePosition();
				double gyro_correction = 0.3 * (-19 - navx_.getAngle());
				
				double left_value = (0.0003 * left_distance_error);
				if(Math.abs(left_value) > 0.40) left_value = Math.signum(left_value) * 0.40;
				double right_value = (0.0003 * right_distance_error);
				if(Math.abs(right_value) > 0.40) right_value = Math.signum(left_value) * 0.40;
				
				gyro_correction = (Math.abs(gyro_correction) > 0.1) ? Math.signum(gyro_correction) * 0.1 : gyro_correction;
				
				drive_.leftSideControl(left_value + gyro_correction);
				drive_.rightSideControl(right_value - gyro_correction);
				
				if(wrist_.getWristPosition() < 3000) {
					arm_.setArmPosition(4400);
				}
				wrist_.setWristPosition(1850);
				
				
			} else {
				state = State.SPIT;
				timer.stop();
				timer.reset();
				timer.start();
			}
			break;
			
		case SPIT:
			if(timer.get() <= state.getTime()) {
				double gyro_correction = 0.02 * (15 - navx_.getAngle());
				if(Math.abs(gyro_correction) > 0.50) gyro_correction = 0.50 * Math.signum(gyro_correction);
				drive_.leftSideControl(gyro_correction);
				drive_.rightSideControl(-gyro_correction);

				if(timer.get() >= 1.0) {
					wrist_.outtakeValue(0.75);
				} else {
					wrist_.stop();
				}
			} else {
				state = State.END;
				timer.stop();
				timer.reset();
				timer.start();
			}
			break;
			
		case END:
			drive_.leftSideControl(0.0);
			drive_.rightSideControl(0.0);
			wrist_.stop();
		}
		
		SmartDashboard.putString("Auto Mode", state.name());
		
	}

}