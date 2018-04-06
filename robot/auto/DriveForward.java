package org.usfirst.frc.team5817.robot.auto;

import org.usfirst.frc.team5817.robot.Arm;
import org.usfirst.frc.team5817.robot.Controller;
import org.usfirst.frc.team5817.robot.Drive;
import org.usfirst.frc.team5817.robot.NavxHelper;
import org.usfirst.frc.team5817.robot.Wrist;

import edu.wpi.first.wpilibj.Timer;

public class DriveForward extends AutoMode{
	
	private final Drive drive_ = Drive.getInstance();
	private final Controller driverController_ = Controller.getInstance();
	private final Wrist wrist_ = Wrist.getInstance();
	private final Arm arm_ = Arm.getInstance();
	private final NavxHelper navx_ = NavxHelper.getInstance();
	
	private State state = State.DRIVE_FORWARD;
	
	private Timer timer = new Timer();
	
	private enum State{
		
		DRIVE_FORWARD(1.0),
		STOP(5.0);
		
		double time;
		State(double time) {
			this.time = time;
		}
		
		double getTime() {
			return time;
		}
	}

	@Override
	public void execute() {
		switch(state){
		case DRIVE_FORWARD:
			if(timer.get() < state.getTime()){
				drive_.rightSideControl(0.8);
				drive_.leftSideControl(0.8);
			}else{
				
				state = State.STOP;
				
				timer.stop();
				timer.reset();
				timer.start();
			}
			break;
			
			case STOP:
				if (timer.get()< state.getTime()){
					drive_.rightSideControl(0.0);
					drive_.leftSideControl(0.0);
				}else{
					drive_.rightSideControl(0.0);
					drive_.leftSideControl(0.0);
				}
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
		
		state = State.DRIVE_FORWARD;
		
	}

}
