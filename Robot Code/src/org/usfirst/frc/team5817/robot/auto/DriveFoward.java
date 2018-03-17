package org.usfirst.frc.team5817.robot.auto;

import org.usfirst.frc.team5817.robot.Arm;
import org.usfirst.frc.team5817.robot.Controller;
import org.usfirst.frc.team5817.robot.Drive;
import org.usfirst.frc.team5817.robot.NavxHelper;
import org.usfirst.frc.team5817.robot.Wrist;

import edu.wpi.first.wpilibj.Timer;

public class DriveFoward extends AutoMode{
	
	private final Drive drive_ = Drive.getInstance();
	private final Controller driverController_ = Controller.getInstance();
	private final NavxHelper navx_ = NavxHelper.getInstance();
	
	private State state = State.DRIVE_FORWARD;
	
	private Timer timer = new Timer();
	
	
	
	
	private enum State{
		DRIVE_FORWARD(1.0),
		STOP(1.0);
		double time;
		State(double time)  {
			this.time = time;}


}

	@Override
	public void execute() {
		switch(state){
		case DRIVE_FORWARD:
			drive_.leftSideControl(1.0);
			//drive_.rightsidecontrol(1.0);
		

		state = State.DRIVE_FORWARD;
		timer.stop();
		timer.reset();
		timer.start();
	}
	}

	@Override
	public void initialize() {
	}
		// TODO Auto-generated method stub
		
	}	