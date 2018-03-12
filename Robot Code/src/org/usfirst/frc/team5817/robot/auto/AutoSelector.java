package org.usfirst.frc.team5817.robot.auto;

import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team5817.robot.auto.OneScaleTwoSwitchSameSide;

public class AutoSelector extends AutoMode {
	
	final AutoMode oneScaleTwoSwitchSameSide = new OneScaleTwoSwitchSameSide();
	
	public void selectAuto(){
		
		
		boolean initialized = false;
		String targetOrder;
		
		targetOrder = DriverStation.getInstance().getGameSpecificMessage();
		//if (targetOrder.length() > 0){
			if (targetOrder == "LRL"){
				
				System.out.println("LRL");
				
			}else if(targetOrder == "LLL"){
				
				if (initialized == false){
					oneScaleTwoSwitchSameSide.initialize();
					initialized = true;
				}
				oneScaleTwoSwitchSameSide.execute();
				
			}else if(targetOrder == "RLR"){
				
				System.out.println("RLR");
				
			}else if(targetOrder.charAt(0) == 'L' && targetOrder.charAt(1) == 'L'){
				if (initialized == false){
					oneScaleTwoSwitchSameSide.initialize();
					initialized = true;
				}else{
					oneScaleTwoSwitchSameSide.execute();
				}
				System.out.println("Recieved");
				
			}else{
				
				targetOrder = DriverStation.getInstance().getGameSpecificMessage();
				System.out.println("No Data");
				
			}
		//}else{
			//System.out.println("No Length");
			//System.out.println(targetOrder);
		//}
	}

	@Override
	public void execute() {
		selectAuto();
		
	}

	@Override
	public void initialize() {
		// TODO Auto-generated method stub
		
	}

}
