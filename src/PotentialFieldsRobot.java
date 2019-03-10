import java.awt.Color;
import java.awt.Point;
import java.awt.geom.Line2D;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

import geometry.IntPoint;
import renderables.*;

public class PotentialFieldsRobot {

	//------------//
	// Attributes //
	//------------//

	// Robot:
	private IntPoint coords; // Position of robot
	private double heading; // Robot's heading in radians
	private final int radius; // Size of the robot (Our robot is a circle)
	private  int stepSize = 10; // How far the robot moves each step, in pixels

	// Sensor:
	private final int sensorRange; // Range of sensors
	private final int sensorDensity; // Number of 'lines' the robot uses to see

	// Starting Location:
	private IntPoint startingLocation;

	// Goal:
	private IntPoint goal;
	private int goalRadius;

	// Obstacles:
	private final List<Renderable> obstacles; // All of the obstacles on the map
	private List<IntPoint> visibleObstacles;

	// Sample:
	private int sampleSize;
	private final int sampleSizeDefault;

	// Image:
	private final RenderableImg robotPic;
	private final RenderablePoint robotPicAlt;

	// Arcs:
	private MyArc firstArc;
	private MyArc secondArc;
	private MyArc thirdArc;

	//private Vector lastMove;

//	private final int vistedScorePower;
//	private final int goalScoreFactor;
	private final int visitedBoxSize;

	private boolean fractionalProgress;

	private final int VISITED_HISTOGRAM_LENGTH;
	private final int VISITED_HISTOGRAM_HEIGHT;

	private final int[][] visitedHistogram;
	private boolean winding = false; //TODO


	//-------------//
	// Constructor //
	//-------------//

	/**
	 * Set the robot moving towards a goal on the screen with set radius, step size,
	 * etc.
	 * 
	 * @param imagePath the on-disk location of the image to use for the robot, or null for default
	 * @param startingLocation the coordinates of the starting point
	 * @param goalLocation the coordinates of the goal
	 * @param radius the radius of the robot
	 * @param sensorRange how far the robot can 'see'
	 * @param sensorDensity the number of sensor lines the robot can use
	 * @param goalRadius the width of the goal
	 * @param obstacles a list of all the obstacles on the map
	 * @param box The size of the box, which will be used for checking the visited histogram.
	 * @param headingR The initial heading.
	 * @param fractionalProgress The boolean value for the fractional progress option.
	 */
	public PotentialFieldsRobot(String imagePath, IntPoint startingLocation, IntPoint goalLocation, int radius,
			int sensorRange, int sensorDensity, int goalRadius, List<Renderable> obstacles,/* int power, int goal,*/ int box, double headingR, boolean fractionalProgress) {
		if (imagePath == null) {
			robotPic = null;
			robotPicAlt = new RenderablePoint(startingLocation.x, startingLocation.y);
			robotPicAlt.setProperties(Color.RED, (float) radius * 2);
		} else {
			robotPicAlt = null;

			robotPic = new RenderableImg("images" + File.separator + imagePath, startingLocation.x, startingLocation.y, radius * 2, radius * 2);
		}

		this.startingLocation = startingLocation;

		this.coords = new IntPoint(startingLocation.x, startingLocation.y);
        heading = headingR;

        // set the boolean value for the fractional progress
        this.fractionalProgress = fractionalProgress;


		this.radius = radius;
		this.sensorRange = sensorRange;
		this.sensorDensity = sensorDensity;
		this.sampleSizeDefault = 2 * radius;
		this.goal = goalLocation;
		this.goalRadius = goalRadius;
		this.obstacles = obstacles;


		// visited histogram:

		int totalX = Math.abs(goalLocation.x - startingLocation.x);
		int totalY = Math.abs(goalLocation.y - startingLocation.y);

		VISITED_HISTOGRAM_LENGTH = totalX / box;
		VISITED_HISTOGRAM_HEIGHT = totalY / box;
		
		visitedBoxSize = box;

		this.visitedHistogram = new int[VISITED_HISTOGRAM_LENGTH][VISITED_HISTOGRAM_HEIGHT];
	}


	/**
	 * The move method of the arc planner.
	 * 
	 * @return True if the move is successful, false if there are no viable moves.
	 */
	public boolean ArcMove() {
		IntPoint moveTo;

		if (this.fractionalProgress) {
			moveTo = evaluateSamplePointsArcForFractionalProgress();
		} else {
			moveTo = evaluateSamplePointsArc(); // Pick a sample point to move towards
		}


		if (moveTo == null)
			return false;

		setArcs(get3Arcs(moveTo, true));

		Vector robotPos = new Vector(coords.x, coords.y);
		Vector samplePos = new Vector(moveTo.x, moveTo.y);


		double theta = Calculator.getTheta(heading, robotPos, samplePos);

		Vector chord = samplePos.diff(robotPos);

		double chordLength = chord.getMagnitude();
		double curvature = Calculator.getCurvature(theta, chordLength);
		double arcLength = Calculator.getArcLength(theta, curvature, chordLength);
		double headingChange =  getHeadingChangeInterval(theta, arcLength);

		moveTowards(heading + headingChange);

		return true;
	}


	/**
	 * Creates a 3 arcs system.
	 * 
	 * @param moveTo sample point we want to move to
	 * @return 3 arcs system.
	 */
	private ArcSet get3Arcs(IntPoint moveTo, boolean draw) {
		Vector robotPos = new Vector(coords.x, coords.y);
		Vector samplePos = new Vector(moveTo.x, moveTo.y);
		Vector goalPos = new Vector(goal.x, goal.y);

		// First arc  :angle  between current heading and thesample point :
		double theta = Calculator.getTheta(heading, robotPos, samplePos);

		// Direction at sample calculations:
		double headingAtSample = Calculator.mod((heading + 2.0* theta), (2*Math.PI));

		Vector segmentFromSampleToGoal = goalPos.diff(samplePos);
		Vector directionAtSample = new Vector(Math.cos(headingAtSample), Math.sin(headingAtSample));

		// Calculate the 3 angles:
		double alpha = Calculator.getAngleBetweenVector(directionAtSample, segmentFromSampleToGoal);
		double absAlpha = Math.abs(alpha);
		double beta = 0.5837 * alpha;
		double absBeta = Math.abs(beta);
		double absGamma = (Math.PI - .5*absAlpha - .5*absBeta);

		// Calculate chords and arcs lengths:
		double secondChordLength = segmentFromSampleToGoal.getMagnitude() * Math.sin(0.5 * absBeta) / Math.sin(absGamma);

	    if (absAlpha == 0 || absBeta == 0) {
	    	secondChordLength  =    goalPos.diff(samplePos).getMagnitude()/2.0;   
	    }

		Vector posV = getPosV(samplePos, secondChordLength, headingAtSample  );


		double headingAtV   =  Calculator.mod((headingAtSample +2*Calculator.getTheta(headingAtSample, samplePos, posV) ), (2*Math.PI));


		MyArc first = new MyArc(new MyPoint(robotPos), new MyPoint(samplePos), heading  ,draw );

		MyArc second = new MyArc(new MyPoint(samplePos), new MyPoint(posV), headingAtSample  ,draw );
          
		MyArc third = new MyArc(new MyPoint(posV), new MyPoint(goalPos), headingAtV, draw );

		return new ArcSet(first, second, third);
	}


    /**
     * Compute the  point  that seperate the  secound arc from the  third arc    
     */
    private Vector getPosV(Vector samplePos, double secondChordLength ,double headingAtSamplex ) {
		Vector xAxis = new Vector (secondChordLength, 0);

        double directedAngle3 = Calculator.getTheta(headingAtSamplex, new Vector( coords.x  ,coords.y ) ,new Vector(goal.x , goal.y)  )  /2.0; 

        Vector Rotated =  new Vector (xAxis.x * Math.cos(headingAtSamplex+ directedAngle3  ) -xAxis.y* Math.sin(headingAtSamplex+ directedAngle3)     , xAxis.x * Math.sin(headingAtSamplex+ directedAngle3) +xAxis.y* Math.sin(headingAtSamplex+ directedAngle3));
        Vector posV  =  samplePos.add(  Rotated );  
   
        return posV;
	}



	private void setArcs(ArcSet arcs) {
		firstArc = arcs.firstArc;
		secondArc = arcs.secondArc;
		thirdArc = arcs.thirdArc;
	}


	private double getHeadingChangeInterval(double theta, double arcLength) {
		return stepSize * 2.0 * theta / arcLength;
	}


    /**
     * compute the Obstacle potential for point P  
     * @return Obstacle potential 
     */
    private double getObstaclePotential(IntPoint p) {

        double[] obsDists = new double[visibleObstacles.size()];

		for (int i = 0; i < visibleObstacles.size(); i++) {
			// Distance is set to 0 if it's closer than the radius to the obstacle
			double distanceFromObstacle = distance(p, visibleObstacles.get(i)) - radius;
			obsDists[i] = distanceFromObstacle <= 0 ? 0 : distanceFromObstacle / 100;
		}

        double obsField = 0;

        for (int i = 0; i < visibleObstacles.size(); i++) {
			if (obsDists[i] <= 0) {
				obsField = Double.MAX_VALUE;
				break;
			} else if (obsDists[i] > sensorRange) {
				continue;
			}

			obsField += Math.pow(Math.E, -1 / ((sensorRange) - obsDists[i])) / (obsDists[i]);
		}

        return obsField;
    }


	/**
	 * Get the potential field at point p. The lower the value returned, the better the point is as a move.
	 *
	 * @param p the point to evaluate
	 * @return The value of the point
	 */
	private double evalMoveArc(IntPoint p, IntPoint goal) {
		ArcSet arcs = get3Arcs(p, false);

		// Everything is divided by 10 because otherwise the numbers get too big
		double goalDist = (arcs.getTotalLengthOfArcs() - radius) / 100;

		double[] obsDists = new double[visibleObstacles.size()];

		for (int i = 0; i < visibleObstacles.size(); i++) {
			// Distance is set to 0 if it's closer than the radius to the obstacle
			double distanceFromObstacle = distance(p, visibleObstacles.get(i)) - radius;
			obsDists[i] = distanceFromObstacle <= 0 ? 0 : distanceFromObstacle / 100;
		}

		// Calculate field power - x^2 so value gets small as distance decreases
		double goalField = Math.pow(goalDist, 2);


		// obsField power is sum of all obstacles, and gets v. large as distance decreases and vice versa
		double obsField = 0;

		for (int i = 0; i < visibleObstacles.size(); i++) {
			if (obsDists[i] <= 0) {
				obsField = Double.MAX_VALUE;
				break;
			} else if (obsDists[i] > sensorRange) {
				continue;
			}

			obsField += Math.pow(Math.E, -1 / ((sensorRange) - obsDists[i])) / (obsDists[i]);
		}


		double totalScore = 10 * goalField + Math.pow(2*radius,2)*4750*obsField / (sensorDensity*sensorRange);

		return totalScore;
	}


	/**
	 * Evaluate all of the robot's potential movement positions & return the best.
	 * 
	 * @return The most valuable point
	 */
	private IntPoint evaluateSamplePointsArc() {
		List<IntPoint> moves = getSamplePoints();

		// If there's no moves that doesn't go through obstacles, quit
		if (moves.isEmpty()) {
			return null;
		}


		// Value of moves is a function of distance from goal & distance from detected objects
		double[] moveValues = new double[moves.size()];

		for (int i = 0; i < moves.size(); i++) {
			moveValues[i] = evalMoveArc(moves.get(i), this.goal);
		}

		return moves.get(minIndex(moveValues)); // Return the lowest valued move
	}


	//TODO --------------------------------------------------------------------------------

	private IntPoint evaluateSamplePointsArcForFractionalProgress() {
		List<IntPoint> moves = getSamplePoints();

		// If there's no moves that doesn't go through obstacles, quit
		if (moves.isEmpty()) {
			return null;
		}

		// An array to store the values of p/(p+f)
		double[] moveValues = new double[moves.size()];

		// An array to store the obstacle potentials
		double[] obstaclePotentials = new double[moves.size()];

		iterateMovePointsToEvaluate(moves, moveValues, obstaclePotentials);

		int minIndex = minIndex(moveValues);

		int minIndexOfObstacle = minIndex(obstaclePotentials);
		int maxIndexOfObstacle = maxIndex(obstaclePotentials);

		//check if the robot is in the winding mode
		if (winding) {
			moves.remove(maxIndexOfObstacle);

			double[] newMoveVals = new double[moves.size()];

			double[] newObstaclePotentials = new double[moves.size()];

			iterateMovePointsToEvaluate(moves, newMoveVals, newObstaclePotentials);

			minIndexOfObstacle = minIndex(newObstaclePotentials);

			if (minIndexOfObstacle == maxIndex(newMoveVals)) {
				System.out.println("hey!");
				winding = false;
			}

			return moves.get(minIndexOfObstacle);

		} else if (minIndex == maxIndexOfObstacle) {
			System.out.println("hello");
			winding = true;
			return moves.get(minIndexOfObstacle);
		}

		return moves.get(minIndex);
	}


	private int getIndexOfNextBestSamplePoint(double[] arr, double cur, int indexOfBiggest) {
		double val = arr[indexOfBiggest];
		int index = indexOfBiggest;

		for (int i = 0; i < arr.length; i++) {
			if (arr[i] > cur && arr[i] < val) {
				val = arr[i];
			}
		}

		return index;
	}

	/**
	 * TODO
	 * @param moves
	 * @param moveValues
	 * @param obstaclePotentials
	 */
	private void iterateMovePointsToEvaluate(List<IntPoint> moves, double[] moveValues, double[] obstaclePotentials) {
		// use for loop to get all IntPoint instances from the list.
		for (int i = 0; i < moves.size(); i++) {
			IntPoint samplePoint = moves.get(i);

			ArcSet set = get3Arcs(samplePoint, false);
			
			//get the obstacle potential of the sample point
			double obstaclePotential = getObstaclePotential(samplePoint);
			obstaclePotentials[i] = obstaclePotential;

			// p = the length of the first arc to the sample point
			double p = set.firstArc.arcLength;

			// f = total length of 3 arcs + obstacle potential
			double f = set.getTotalLengthOfArcs() + obstaclePotential;

			double sum = p + f;

			moveValues[i] = f / (sum);
		}
	}
	
	//TODO -------------------------------------------------------------------------------- 


	/**
	 * Move the robot 1 step towards the goal (point of least potential resistance)
	 * 
	 * @return True if the move is successful, false if there are no viable moves.
	 */
	public boolean move () {
		IntPoint moveTo = evaluateSamplePoints(); // Pick a sample point to move towards
		if (moveTo == null)
			return false;


		IntPoint makeMove = evaluateMovePoints(moveTo); // Find the best move point using current sample as goal TODO: what if another sample point could be used and fail with this one?

		if (makeMove == null)
			return false;


        setArcs(get3Arcs(moveTo, true)); 

		double newHeading = calculateHeading(makeMove);

		moveTowards(newHeading); // Make the move

		return true;
	}


	/**
	 * Have the robot move along a certain heading
	 * 
	 * @param newHeading
	 *            The heading to move along.
	 **/
	private void moveTowards(double newHeading) {
		double deltaX = stepSize * Math.cos(newHeading);
		double deltaY = stepSize * Math.sin(newHeading);
		coords.x += (int) deltaX;
		coords.y += (int) deltaY;
                
		if (robotPic == null) {
			robotPicAlt.x = coords.x;
			robotPicAlt.y = coords.y;
		} else {
			robotPic.x += coords.x;
			robotPic.y += coords.y;
		}

		this.heading =   Calculator.mod(newHeading, Math.PI * 2);
	}


	/**
	 * Find the heading that the robot must move to in order to reach a certain
	 * point. If the angle is greater than 60 degrees, truncate it to 60 degrees,=.
	 * 
	 * @param dest the destination point
	 **/
	public  double calculateHeading(IntPoint dest) {
		double grad = Math.abs(((double) dest.y - (double) coords.y) / ((double) dest.x - (double) coords.x));
		double angle = Math.atan(grad);

		if (dest.x - coords.x < 0) {
			if (dest.y - coords.y < 0) {
				angle = Math.PI + angle;
			} else {
				angle = Math.PI - angle;
			}
		} else {
			if (dest.y - coords.y < 0) {
				angle = (Math.PI * 2) - angle;
			}
		}

		return angle;
	}


	/**
	 * Evaluate all of the robot's potential movement positions & return the best.
	 * 
	 * @return The most valuable point
	 */
	private IntPoint evaluateSamplePoints() {
		List<IntPoint> moves = getSamplePoints();

		// If there's no moves that doesn't go through obstacles, quit
		if (moves.isEmpty()) {
			return null;
		}

		// Value of moves is a function of distance from goal & distance from detected objects
		double[] moveValues = new double[moves.size()];

		for (int i = 0; i < moves.size(); i++) {
			moveValues[i] = evalMove(moves.get(i), this.goal);
		}

		return moves.get(minIndex(moveValues)); // Return the lowest valued move
	}


	/**
	 * Evaluate all of the robot's potential movement positions & return the best.
	 * 
	 * @return The most valuable point
	 */
	private IntPoint evaluateMovePoints(IntPoint goal) {
		List<IntPoint> moves = getMoveablePoints();

		// If there's no moves that don't go through obstacles, quit
		if (moves.isEmpty()) {
			return null;
		}


		// Value of moves is a function of distance from goal & distance from detected objects
		double[] moveValues = new double[moves.size()];

		for (int i = 0; i < moves.size(); i++) {
			moveValues[i] = evalMove(moves.get(i), goal);
		}

		return moves.get(minIndex(moveValues)); // Return the lowest valued move
	}


	/**
	 * Get the potential field at point p. The lower the value returned, the better
	 * the point is as a move.
	 * 
	 * @param p the point to evaluate
	 * @return The value of the point
	 */
	private double evalMove(IntPoint p, IntPoint goal) {
		// Get distances to goal & all visible objects
		// Everything is divided by 10 because otherwise the numbers get too big
		double goalDist = (distance(p, goal) - radius) / 10;

		double[] obsDists = new double[visibleObstacles.size()];

		for (int i = 0; i < visibleObstacles.size(); i++) {
			// Distance is set to 0 if it's closer than the radius to the obstacle
			double distanceFromObstacle = distance(p, visibleObstacles.get(i)) - radius;
			obsDists[i] = distanceFromObstacle <= 0 ? 0 : distanceFromObstacle / 10;
		}

		// Calculate field power - x^2 so value gets small as distance decreases
		double goalField = Math.pow(goalDist, 2);

		// obsField power is sum of all obstacles, and gets v. large as distance decreases and vice versa
		double obsField = 0;

		for (int i = 0; i < visibleObstacles.size(); i++) {
			if (obsDists[i] <= 0) {
				obsField = Double.MAX_VALUE;
				break;
			} else if (obsDists[i] > sensorRange) {
				continue;
			}

			obsField += Math.pow(Math.E, -1 / ((sensorRange) - obsDists[i])) / (obsDists[i]);
		}

		return 10 * goalField + Math.pow(2 * radius, 2) * 4750 * obsField / (sensorDensity * sensorRange);
	}


	/**
	 * Get all of the points the robot can move to - the robot moves 10 pixels
	 * forward, and can turn up to 12 degrees in either direction to simulate
	 * continuous movement.
	 */
	public List<IntPoint> getMoveablePoints() {
		List<IntPoint> moveablePoints = new ArrayList<IntPoint>(5);

		double angleBetween = Math.toRadians(3);
		double currentAngle = mod(heading - Math.toRadians(12), 2 * Math.PI);

		for (int i = 0; i < 9; i++) {
			// Only make this a 'moveable' point if it does not touch an obstacle
			Line2D.Double line = new Line2D.Double();
			IntPoint p2 = getPointTowards(currentAngle, stepSize);

			line.setLine(coords.x, coords.y, p2.x, p2.y);

			// Check if this line intersects an obstacle, and if so, don't add it
			boolean crash = false;

			for (IntPoint p : visibleObstacles) {
				if (distance(p, p2) <= radius) {
					crash = true;
				}
			}

			if (intersects(line) == null && !crash) {
				moveablePoints.add(p2);
			}

			currentAngle = mod((currentAngle + angleBetween), 2 * Math.PI);
		}

		return moveablePoints;
	}


	/**
	 * Get a list of all the sample points evenly distributed in a 180-degree arc in
	 * front of the robot
	 **/
	public List<IntPoint> getSamplePoints() {
		List<IntPoint> moveablePoints = new ArrayList<IntPoint>(sensorDensity);

		double angleInterval = Math.PI / (sensorDensity - 1);
		double currentAngle = mod(heading - Math.PI / 2, 2 * Math.PI);
        double  distToObstical = distanceToClosestObstacle() ;   

        sampleSize  = sampleSizeDefault ;


        if (distToObstical !=0 && sampleSize > distToObstical)
            sampleSize =  (int) (distToObstical/2.0 );


        if ( distance(goal, coords )/2.0 < sampleSize )
            sampleSize = (int) (distance(goal, coords ) / 2.0);

        if (sampleSize < radius)
            sampleSize = radius + 1; 

        if(sampleSize <= 10) 
            sampleSize = 10; 


		for (int i = 0; i < sensorDensity; i++) {
			// Only make this a 'moveable' point if it does not touch an obstacle
			Line2D.Double line = new Line2D.Double();
			IntPoint p2 = getPointTowards(currentAngle, sampleSize);

			line.setLine(coords.x, coords.y, p2.x, p2.y);
			
			moveablePoints.add(p2);
			currentAngle += angleInterval;
		}


        stepSize = (int) (sampleSize / 2.0);

         if( stepSize <= 1)
        	 stepSize = 2;

         if( stepSize > 10)
        	 stepSize = 10 ;  
                   
		return moveablePoints;
	}


	/**
	 * Get all of the points the robot can move to - the number is equal to the
	 * robot's sensor density spread equally in a 180 degree arc in front of the
	 * robot. Additionally, calculate if a sensor hits an obstacle and make a note
	 * of the collision point
	 **/
	public List<IntPoint> getSensorablePoints() {
		List<IntPoint> sensorablePoints = new ArrayList<IntPoint>(sensorDensity);

		visibleObstacles = new ArrayList<IntPoint>();

		double angleBetween = Math.PI / (sensorDensity - 1);
		double currentAngle = mod(heading - Math.PI / 2, 2 * Math.PI);

		for (int i = 0; i < sensorDensity; i++) {
			int sensorRange = this.sensorRange;

			// Check for intersecting obstacles
			IntPoint edge = getPointTowards(currentAngle, sensorRange);

			Line2D.Double sensorLine = new Line2D.Double(new Point(coords.x, coords.y), new Point(edge.x, edge.y));
			
			IntPoint intersection = intersects(sensorLine);

			if (intersection != null) {
				sensorRange = (int) distance(intersection, coords);
				visibleObstacles.add(intersection);
			}

			sensorablePoints.add(getPointTowards(currentAngle, sensorRange));
			currentAngle += angleBetween;
		}

		return sensorablePoints;
	}


	/**
	 * Get the closest point where this line crosses an obstacle - this varies based
	 * on the obstacle type In general, this is achieved by turning the obstacle
	 * into a series of lines and calling getIntersectionPoint() on the target line
	 * and each of the polygon's lines. Once all intersection points are found, the
	 * closest to the robot is returned. It is assumed all polygons are convex.
	 */
	private IntPoint intersects(Line2D.Double line) {
		ArrayList<IntPoint> intersections = new ArrayList<IntPoint>();

		for (Renderable obstacle : obstacles) {

			if (obstacle.getClass() == RenderablePolyline.class) {
				ArrayList<Integer> xs = ((RenderablePolyline) obstacle).xPoints;
				ArrayList<Integer> ys = ((RenderablePolyline) obstacle).yPoints;

				for (int i = 0; i < xs.size() - 1; i++) {
					Line2D.Double obsLine = new Line2D.Double(xs.get(i), ys.get(i), xs.get(i + 1), ys.get(i + 1));
					IntPoint intersect = getIntersectionPoint(line, obsLine);

					if (intersect != null)
						intersections.add(intersect);
				}

			} else if (obstacle.getClass() == RenderableRectangle.class) {
				/*
				 * Rectangle is treated like a polygon but since because it's a different class
				 * it has to be handled separately - we've got to construct the polypoints
				 * separately (annoyingly)
				 */
				ArrayList<Integer> xs = new ArrayList<Integer>();
				ArrayList<Integer> ys = new ArrayList<Integer>();

				xs.add(((RenderableRectangle) obstacle).bottomLeftX);
				xs.add(((RenderableRectangle) obstacle).bottomLeftX);
				xs.add(((RenderableRectangle) obstacle).bottomLeftX + ((RenderableRectangle) obstacle).width);
				xs.add(((RenderableRectangle) obstacle).bottomLeftX + ((RenderableRectangle) obstacle).width);

				ys.add(((RenderableRectangle) obstacle).bottomLeftY);
				ys.add(((RenderableRectangle) obstacle).bottomLeftY + ((RenderableRectangle) obstacle).height);
				ys.add(((RenderableRectangle) obstacle).bottomLeftY + ((RenderableRectangle) obstacle).height);
				ys.add(((RenderableRectangle) obstacle).bottomLeftY);

				for (int i = 0; i < xs.size(); i++) {
					Line2D.Double obsLine = new Line2D.Double(xs.get(i), ys.get(i), xs.get((i + 1) % xs.size()),
							ys.get((i + 1) % ys.size()));

					IntPoint intersect = getIntersectionPoint(line, obsLine);

					if (intersect != null)
						intersections.add(intersect);
				}

			} else if (obstacle.getClass() == RenderablePolygon.class) {
				ArrayList<Integer> xs = ((RenderablePolygon) obstacle).xPoints;
				ArrayList<Integer> ys = ((RenderablePolygon) obstacle).yPoints;

				for (int i = 0; i < xs.size(); i++) {
					Line2D.Double obsLine = new Line2D.Double(xs.get(i), ys.get(i), xs.get((i + 1) % xs.size()), ys.get((i + 1) % ys.size()));

					IntPoint intersect = getIntersectionPoint(line, obsLine);

					if (intersect != null)
						intersections.add(intersect);
				}

			} else if (obstacle.getClass() == RenderableOval.class) {

				// ovals are treated as their bounding polygons (90-sided) and they have to be
				// circles
				ArrayList<Integer> xs = new ArrayList<Integer>();
				ArrayList<Integer> ys = new ArrayList<Integer>();

				RenderableOval roval = (RenderableOval) obstacle;


				for (int i = 0; i < 90; i++) {
					int trigPoint = (int) (roval.width / 2 * Math.cos(i * Math.PI / 45));

					xs.add(roval.centreX + trigPoint);
				}

				for (int i = 0; i < 90; i++) {
					int trigPoint = (int) (roval.width / 2 * Math.sin(i * Math.PI / 45));

					ys.add(roval.centreY + trigPoint);
				}

				for (int i = 0; i < xs.size(); i++) {
					Line2D.Double obsLine = new Line2D.Double(xs.get(i), ys.get(i), xs.get((i + 1) % xs.size()), ys.get((i + 1) % ys.size()));
					
					IntPoint intersect = getIntersectionPoint(line, obsLine);
					
					if (intersect != null)
						intersections.add(intersect);
				}

			}
		}

		return intersections.isEmpty() ? null : lowestDist(intersections);
	}


	/**
	 * Get the closest point to the robot's coords
	 * 
	 * @param points
	 *            A list of point
	 *            
	 * @return The point with the smallest distance from the robot
	 **/
	private IntPoint lowestDist(ArrayList<IntPoint> points) {
		int lowest = 0;

		for (int i = 0; i < points.size(); i++) {
			if (distance(points.get(i), coords) < distance(points.get(lowest), coords))
				lowest = i;
		}

		return points.get(lowest);
	}


	/**
	 * Get the point 'step' pixels along the given heading from the robot's position
	 * 
	 * @param heading
	 *            The heading to move along
	 * @param step
	 *            The distance to travel along that heading
	 **/
	private IntPoint getPointTowards(double heading, int step) {
		int length = (int) (step * Math.cos(heading));
		int height = (int) (step * Math.sin(heading));

		return new IntPoint(coords.x + length, coords.y + height);
	}


	/**
	 * Get the position of the minimum value in the array.
	 *
	 * @param nums A list of numbers
	 * @return The index of the minimum number in the list.
	 */
	private int minIndex(double[] nums) {
		int minIndex = 0;

		for (int i = 1; i < nums.length; i++) {
			if (nums[i] < nums[minIndex])
				minIndex = i;
		}

		return minIndex;
	}


	/**
	 * Gets the index of the maximum number in the given array.
	 *
	 * @param nums
	 * @return
	 */
	int maxIndex(double[] nums) {
		int index = 0;

		for (int i = 1; i < nums.length; i++) {
			if (nums[i] > nums[index])
				index = i;
		}

		return index;
	}

	/**
	 * Get the distance between two points.
	 **/
	private static double distance(IntPoint a, IntPoint b) {
		return Math.sqrt(Math.pow((a.x - b.x), 2) + Math.pow((a.y - b.y), 2));
	}


	/**
	 * Check if the robot falls within the goal radius.
	 **/
	public boolean inGoal() {
		return distance(coords, goal) < goalRadius + radius;
	}


	/**
	 * Calculate the intersection point of two lines, or return null if there is no
	 * intersection.
	 * 
	 * @param line1
	 *            The first line
	 * @param line2
	 *            The second line
	 * @return The point of intersection, or null.
	 */
	private static IntPoint getIntersectionPoint(Line2D.Double line1, Line2D.Double line2) {
		if (!line1.intersectsLine(line2))
			return null;

		double px = line1.getX1(), py = line1.getY1(), rx = line1.getX2() - px, ry = line1.getY2() - py;
		double qx = line2.getX1(), qy = line2.getY1(), sx = line2.getX2() - qx, sy = line2.getY2() - qy;

		double det = sx * ry - sy * rx;

		if (det == 0) {
			return null;
		} else {
			double z = (sx * (qy - py) + sy * (px - qx)) / det;

			if (z == 0 || z == 1)
				return null; // intersection at end point

			return new IntPoint((int) (px + z * rx), (int) (py + z * ry));
		}
	}


	/**
	 * Calculate a % b, but always result in a positive answer - java's default
	 * syntax returns a negative number if the dividend is negative, which is
	 * unhelpful when calculations are performed between 0 and 2PI, rather than
	 * -PI and PI.
	 **/
	private static double mod(double a, double b) {
		return ((a % b) + b) % b;
	}


	/**
	 * Find the distance from the robot to the closest visible obstacle, or some
	 * default if none are visible
	 **/
	private int distanceToClosestObstacle() {
		if (visibleObstacles.isEmpty())
			return 0;
		int closestIndex = 0;
		for (int i = 0; i < visibleObstacles.size(); i++)
			if (distance(coords, visibleObstacles.get(i)) < distance(coords, visibleObstacles.get(closestIndex)))
				closestIndex = i;
		return (int) Math.round(distance(coords, visibleObstacles.get(closestIndex)));
	}



	// Getters:

	public int getStepSize() {
		return this.stepSize;
	}

	public IntPoint getPosition() {
		return coords;
	}

	/**
	 * Return the image representing the robot. This can be a red blob, or a picture
	 * depending on the options enabled.
	 **/
	public Renderable getImage() {
		if (robotPic != null) {
			robotPic.x = coords.x - radius;
			robotPic.y = coords.y - radius;
			return robotPic;
		} else {
			return robotPicAlt;
		}
	}

	public MyArc getFirstArc() { //TODO: null pointer exception moving between plannars halway
		return firstArc;
	}

	public MyArc getSecondArc() {
		return secondArc;
	}
	
	public MyArc getThirdArc() {
		return thirdArc;
	}

	public class ArcSet {
		MyArc firstArc;
		MyArc secondArc;
		MyArc thirdArc;
		
		public ArcSet(MyArc f, MyArc s, MyArc t) {
			firstArc = f;
			secondArc = s;
			thirdArc = t;
		}

		/**
		 * This method returns to total length of the 3 arcs.
		 * @return the total length of the 3 arcs
		 */
		public double getTotalLengthOfArcs() {
			return firstArc.arcLength + secondArc.arcLength + thirdArc.arcLength;
		}
	}
        
    public void setGoal(IntPoint newGoal) {
		this.goal = newGoal;
	}
	
	public void setGoalRadius(int rad) {
		this.goalRadius = rad;
	}

    public void setHeading  ( double  h ){
        this.heading= h ; 
    }

}
