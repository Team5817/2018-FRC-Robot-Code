package org.usfirst.frc.team5817.robot;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.wpilibj.SerialPort;

public class NavxHelper {

	private final AHRS gyro;
	
	private static NavxHelper instance_;
	
	private NavxHelper() {
		byte update_rate = 50;
		gyro = new AHRS(SerialPort.Port.kUSB, SerialDataType.kProcessedData, update_rate);
	}

	public static NavxHelper getInstance() {
		if(instance_ == null) {
			instance_ = new NavxHelper();
		}
		return instance_;
	}
	
	public double getAngle() {
		return gyro.getYaw();
	}
	
	public void zeroAngle() {
		gyro.zeroYaw();
	}
	
	public boolean isConnected() {
		return gyro.isConnected();
	}
	
	
}