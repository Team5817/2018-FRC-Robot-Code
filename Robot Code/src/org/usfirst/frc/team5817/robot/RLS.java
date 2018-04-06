package org.usfirst.frc.team5817.robot;



/**
 * MIT License

 * Copyright (c) 2018 Mitchel Stokes
	
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
	
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
	
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *
 *
 * Robot Localization System - Configure the needed parameters using the configure() method.
 * It is important to note that all of the calculations are done without units; this means that
 * you the user will need to convert the raw encoder readings to distance (i.e. by calculating
 * gearbox reductons, wheel diameter, etc.) and feed those values into the RLS class. Because the 
 * code has no units, you are free to use whatever units you like, as long as you keep them consistent
 * between all values.
 * 
 * @author Mitchel Stokes
 */
public class RLS {

	
	private static RLS instance_;
	
	private double theta_last = 0.0;
	private double x_last = 0.0;
	private double y_last = 0.0;
	private double width = 0.0;
	
	private boolean is_configured = false;
	
	/**
	 * Private constructor to only allow singleton construction
	*/
	private RLS() {}
	
	
	/**
	 * Returns the single instance of RLS, and creates one if one does not exist yet.
	 * 
	 * @return The single instance of RLS
	 */
	public static RLS getInstance() {
		if(instance_ == null) {
			instance_ = new RLS();
		}
		return instance_;
	}
	
	
	/**
	 * Configures the RLS system with the needed parameters. The update() function will not
	 * work until you run this function with valid parameters.
	 * 
	 * @param width Effective wheelbase of the robot in desired units.
	 * @param x_0 Starting X position of the robot, in the same units as wheelbase width.
	 * @param y_0 Starting Y position of the robot, in the same units as wheelbase width.
	 * @param theta_0 Starting angle of the robot, in degrees.
	 */
	public void configure(double width, double x_0, double y_0, double theta_0) {
		this.width = width;
		this.x_last = x_0;
		this.y_last = x_0;
		this.theta_last = theta_0;
		
		is_configured = true;
	}
	
	
	/**
	 * Updates the position of the robot using left and right-side position deltas since last update.
	 * The update functions uses a constant-curvature model of the robot's trajectory over time. It is
	 * unknown how accurate this will be, but there will be some error associated with this method of
	 * approximation.
	 * 
	 * @param left_delta Left side distance delta since last update, in the same units as wheelbase width.
	 * @param right_delta Right side distance delta since last update, in the same units as wheelbase width.
	 */
	public void update(double left_delta, double right_delta) {
		if(is_configured) {
			double theta_delta = (left_delta - right_delta) / width;
			double R = 0.0;
			if(left_delta > right_delta) {
				R = ((right_delta * width) / (left_delta - right_delta)) + (width / 2);
			} else if(left_delta < right_delta) {
				R = ((left_delta * width) / (right_delta - left_delta)) + (width / 2);
			} else {
				R = Double.MAX_VALUE;
			}
			
			double forward_delta_magnitude = R * Math.sin(Math.abs(theta_delta));
			double right_delta_magnitude = R - (R * Math.cos(theta_delta));
			
			double x_delta = (forward_delta_magnitude * Math.cos(Math.toRadians(450 - theta_last))) 
					+ (right_delta_magnitude * Math.cos(Math.toRadians(450 - (theta_last + 90))));
			double y_delta = (forward_delta_magnitude * Math.sin(Math.toRadians(450 - theta_last))) 
					+ (right_delta_magnitude * Math.sin(Math.toRadians(450 - (theta_last + 90))));
			
			theta_last += Math.toDegrees(theta_delta);
			x_last += x_delta;
			y_last += y_delta;
		} else {
			System.out.println("Update failed: RSL not configured");
		}
	}
	
	
	/**
	 * Sets the current X position to the value given (useful for re-zeroing).
	 * 
	 * @param x The new X position
	 */
	public void setX(double x) {
		x_last = x;
	}
	
	
	/**
	 * Sets the current Y position to the value given (useful for re-zeroing).
	 * 
	 * @param x The new Y position
	 */
	public void setY(double y) {
		y_last = y;
	}
	
	
	/**
	 * Sets the current heading to the value given (useful for re-zeroing).
	 * 
	 * @param x The new heading
	 */
	public void setHeading(double heading) {
		theta_last = theta_last;
	}
	
	
	/**
	 * Gets the X position of the robot. Returns 0 if not configured.
	 * 
	 * @return The X position of the robot
	 */
	public double getX() {
		return x_last;
	}
	
	
	/**
	 * Gets the Y position of the robot. Returns 0 if not configured.
	 * 
	 * @return The Y position of the robot
	 */
	public double getY() {
		return y_last;
	}
	
	
	/**
	 * Gets the heading of the robot. Returns 0 if not configured.
	 * 
	 * @return The heading of the robot
	 */
	public double getHeading() {
		return theta_last;
	}
	
	
	/**
	 * Special case - Calls setX(0), setY(0), and setHeading(0) (re-zeroes all values).
	 */
	public void zero() {
		setX(0);
		setY(0);
		setHeading(0);
	}
	
	
	/**
	 * Returns a string status readout of the RLS containing X position, Y position, and heading obtained
	 * using their respective get methods.
	 * 
	 * @return String containing RLS status readout
	 */
	public String toString() {
		return "X Position: " + getX() + "\n    Y Position: " + getY() + "\n    Heading: " + getHeading() + " compass degrees";
	}
	
}