public class Calculator {

	/**
	 * Compute the curvature of the arc.
	 *
	 * @param theta
	 * @param chordLength
	 * @return the curvature of the arc
	 */
	public static double getCurvature(double theta, double chordLength) {
		return Math.abs(2 * Math.sin(Math.abs(theta)) / chordLength);
                
	}


	/**
	 * Compute the Arc length of the arc.
	 *
	 * @param theta
	 * @param curvature
	 * @param chordLength
	 * @return the calculated arc length
	 */
	public static double getArcLength(double theta, double curvature , double chordLength  ) {
        if (theta% (2*Math.PI ) == 0   ) 
            return chordLength; 
        else if ( Math.abs(mod(theta, (2*Math.PI ))) == Math.PI) 
            return 2 * Math.abs(theta) * chordLength;    
        else 
        	return 2 * Math.abs(theta) / curvature;
	}


    /**
     * Compute the signed angle form heading vector to the goal vector.
     *
     * @param heading
     * @param robotPos
     * @param goalPos
     * @return the calculated value of theta
     */
	public static double getTheta(double heading, Vector robotPos, Vector goalPos) {
		Vector headingV = new Vector(Math.cos(heading), Math.sin(heading));
		Vector differenceV = goalPos.diff(robotPos);
		double theta = getAngleBetweenVector(headingV, differenceV);		
		return theta;
	}


	 /**
	  * Compute the unsigned angle form the heading vector to the goal vector . 
	  *
	  * @param headingV the heading vector
	  * @param differenceV the goal vector
	  * @return angle
      */
	public static double getAngleBetweenVector(Vector headingV, Vector differenceV) {

		double test1 = Math.atan2(differenceV.y, differenceV.x);
		double test2 = Math.atan2(headingV.y, headingV.x);

		double angle = test1 - test2;
		if (angle < -Math.PI)
			angle += 2 * Math.PI;
		if (angle >= Math.PI)
			angle -= 2 * Math.PI;
		return angle;
	}
	

	/**
     * Modulus  operation
     *
     * @param a
     * @param b
     * @return The result of ((a % b) + b) % b
     */
	public static double mod(double a, double b) {
		return ((a % b) + b) % b;
	}
	
	
}
