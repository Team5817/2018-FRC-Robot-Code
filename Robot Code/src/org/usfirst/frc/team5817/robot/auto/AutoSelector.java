package org.usfirst.frc.team5817.robot.auto;

import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team5817.robot.auto.OneScaleTwoSwitchSameSide;
import org.usfirst.frc.team5817.robot.auto.OneSwitchOppositeSide;
import org.usfirst.frc.team5817.robot.auto.OneSwitchSameSide;

public class AutoSelector extends AutoMode {
	
	AutoMode selectedAuto;
	
	public void selectAuto(){
		
		
		boolean initialized = false;
		String targetOrder;
		
		targetOrder = DriverStation.getInstance().getGameSpecificMessage();
		//if (targetOrder.length() > 0){
			if (targetOrder.charAt(0) == 'R' && targetOrder.charAt(1) == 'R'){
				
				selectedAuto = new OneSwitchOppositeSide();
				
			}else if(targetOrder.charAt(0) == 'R' && targetOrder.charAt(1) == 'L'){
				
				selectedAuto = new OneSwitchOppositeSide();
				
			}else if(targetOrder.charAt(0) == 'L' && targetOrder.charAt(1) == 'L'){
				
				selectedAuto = new OneScaleTwoSwitchSameSide();

			}else if(targetOrder.charAt(0) == 'L' && targetOrder.charAt(1) == 'R'){
				
				selectedAuto = new OneSwitchSameSide();
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
		selectedAuto.execute();
	}

	@Override
	public void initialize() {
		selectAuto();
		selectedAuto.initialize();
		
	}

}
