package org.usfirst.frc.team5817.robot.auto;

import org.usfirst.frc.team5817.robot.Arm;
import org.usfirst.frc.team5817.robot.Controller;
import org.usfirst.frc.team5817.robot.Drive;
import org.usfirst.frc.team5817.robot.NavxHelper;
import org.usfirst.frc.team5817.robot.Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OneScaleOppositeSide extends AutoMode {
	
	private final Drive drive_ = Drive.getInstance();
	private final Controller driverController_ = Controller.getInstance();
	private final Wrist wrist_ = Wrist.getInstance();
	private final Arm arm_ = Arm.getInstance();
	private final NavxHelper navx_ = NavxHelper.getInstance();
	
	private State state = State.DRIVE_BACKWARDS_1;
	
	private Timer timer = new Timer();
	
	private enum State {
		DRIVE_BACKWARDS_1(3.25),
		TURN_1(0.6),
		DRIVE_BACKWARDS_2(3.5),
		TURN_2(1.25),
		DRIVE_BACKWARDS_AND_RAISE_ARM(3.0),
		SPIT(0.25),
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
		
		state = State.DRIVE_BACKWARDS_1;
	
	}
	
	@Override
	public void execute() {
		switch(state){
		
		case DRIVE_BACKWARDS_1:
			if(timer.get() <= state.getTime()) {
				if(timer.get() <= 0.5) {
					double gyro_correction = 0.04 * navx_.getAngle();
					drive_.leftSideControl((2 * timer.get() * -0.75) - gyro_correction);
					drive_.rightSideControl((2 * timer.get() * -0.75) + gyro_correction);
				} else if(timer.get() <= 0.75) {
					double gyro_correction = 0.01 * navx_.getAngle();
					drive_.leftSideControl(-1.0 - gyro_correction);
					drive_.rightSideControl(-1.0 + gyro_correction);
				} else if(Math.abs(-340000 - drive_.getLeftDrivePosition()) > 150000) {
					double left_distance_error = -340000 - drive_.getLeftDrivePosition();
					double right_distance_error = -340000 - drive_.getRightDrivePosition();
					
					double left_value = 0.0;
					double right_value = 0.0;
					
					if(Math.abs(left_distance_error) < 100000) {
						double gyro_correction = 0.04 * (navx_.getAngle());
						left_value = -1.0 + gyro_correction;
						if(Math.abs(left_value) > 0.95) left_value = Math.signum(left_value) * 0.95;
						right_value = -1.0 - gyro_correction;
						if(Math.abs(right_value) > 0.95) right_value = Math.signum(left_value) * 0.95;
					} else {
						double gyro_correction = 0.01 * (navx_.getAngle());
						left_value = -1.0 - gyro_correction;
						if(Math.abs(left_value) > 0.95) left_value = Math.signum(left_value) * 0.95;
						right_value = -1.0 + gyro_correction;
						if(Math.abs(right_value) > 0.95) right_value = Math.signum(left_value) * 0.95;
					}
					
					drive_.leftSideControl(left_value);
					drive_.rightSideControl(right_value);
				} else {
					double left_distance_error = -340000 - drive_.getLeftDrivePosition();
					double right_distance_error = -340000 - drive_.getRightDrivePosition();
					double gyro_correction = 0.04 * (navx_.getAngle());
					
					double left_value = (0.0002 * left_distance_error) - gyro_correction;
					if(Math.abs(left_value) > 0.25) left_value = Math.signum(left_value) * 0.25;
					double right_value = (0.0002 * right_distance_error) - gyro_correction;
					if(Math.abs(right_value) > 0.25) right_value = Math.signum(left_value) * 0.25;
					
					drive_.leftSideControl(left_value);
					drive_.rightSideControl(right_value);
				}
				
				wrist_.setWristPosition(50);
			} else {
				state = State.TURN_1;
				timer.stop();
				timer.reset();
				timer.start();
			}
			break;
			
		case TURN_1:
			if(timer.get() <= state.getTime()) {
				double gyro_correction = 0.15 * (86 - navx_.getAngle());
				if(Math.abs(gyro_correction) > 0.60) gyro_correction = 0.60 * Math.signum(gyro_correction);
				drive_.leftSideControl(gyro_correction);
				drive_.rightSideControl(-gyro_correction);
			} else {
				state = State.DRIVE_BACKWARDS_2;
				timer.stop();
				timer.reset();
				timer.start();
				drive_.zeroSensors();
			}
			break;
			
		case DRIVE_BACKWARDS_2:
			if(timer.get() <= state.getTime()) {
				if(timer.get() <= 0.5) {
					double gyro_correction = 0.05 * (86 - navx_.getAngle());
					drive_.leftSideControl((2 * timer.get() * -0.75) + gyro_correction);
					drive_.rightSideControl((2 * timer.get() * -0.75) - gyro_correction);
				} else if(timer.get() <= 0.75) {
					double gyro_correction = 0.05 * (86 - navx_.getAngle());
					drive_.leftSideControl(-1.0 + gyro_correction);
					drive_.rightSideControl(-1.0 - gyro_correction);
				} else if(Math.abs(-340000 - drive_.getLeftDrivePosition()) > 125000) {
					double left_distance_error = -260000 - drive_.getLeftDrivePosition(); //-340000
					double right_distance_error = -260000 - drive_.getRightDrivePosition();
					
					double left_value = 0.0;
					double right_value = 0.0;
					
					double gyro_correction = 0.05 * (86 - navx_.getAngle());
					left_value = -1.0 + gyro_correction;
					if(Math.abs(left_value) > 0.95) left_value = Math.signum(left_value) * 0.95;
					right_value = -1.0 - gyro_correction;
					
					drive_.leftSideControl(left_value);
					drive_.rightSideControl(right_value);
				} else {
					double left_distance_error = -260000 - drive_.getLeftDrivePosition(); //-340000
					double right_distance_error = -260000 - drive_.getRightDrivePosition();
					double gyro_correction = 0.05 * (86 - navx_.getAngle());
					
					double left_value = (0.0002 * left_distance_error) + gyro_correction;
					if(Math.abs(left_value) > 0.25) left_value = Math.signum(left_value) * 0.25;
					double right_value = (0.0002 * right_distance_error) - gyro_correction;
					if(Math.abs(right_value) > 0.25) right_value = Math.signum(left_value) * 0.25;
					
					drive_.leftSideControl(left_value);
					drive_.rightSideControl(right_value);
				}
			} else {
				state = State.END;
				timer.stop();
				timer.reset();
				timer.start();
			}
			break;
			
		case TURN_2:
			if(timer.get() <= state.getTime()) {
				double gyro_correction = 0.1 * (-15 - navx_.getAngle());
				if(Math.abs(gyro_correction) > 0.40) gyro_correction = 0.40 * Math.signum(gyro_correction);
				drive_.leftSideControl(gyro_correction);
				drive_.rightSideControl(-gyro_correction);
			} else {
				state = State.DRIVE_BACKWARDS_AND_RAISE_ARM;
				timer.stop();
				timer.reset();
				timer.start();
				drive_.zeroSensors();
			}
			break;
			
		case DRIVE_BACKWARDS_AND_RAISE_ARM:
			if(timer.get() <= state.getTime()) {
				double left_distance_error = -90000 - drive_.getLeftDrivePosition();
				double right_distance_error = -90000 - drive_.getRightDrivePosition();
				double gyro_correction = 0.3 * (-15 - navx_.getAngle());
				
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
				wrist_.outtakeValue(0.75);
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