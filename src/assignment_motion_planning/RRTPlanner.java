/**
 * This algorithm is from the following paper
 * Steven M. LaValle and James  Kuffner Jr.,
 * Randomized Kinodynamic Planning, 
 * The International Journal of Robotics Research 20 (5), pp. 378–400.
 * http://dx.doi.org/10.1177/02783640122067453
 */

package assignment_motion_planning;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import javafx.geometry.Point2D;
import javafx.util.Pair;

public class RRTPlanner extends MotionPlanner {
    private static final double DEFAULT_DELTA = 0.1;  // Duration for the control
    
    // representation of a tree
    private HashMap<Vector, Edge> parents;
    // option for random delta
    private boolean randDelta;
    // number of nearest neighbors
    private int k;
    // option for goal bias
    private boolean goalBias;
    
    private class Edge {
    	private Vector parent;
    	private Vector control;
    	private double duration;
    	/** 
    	 * Constructor
    	 * @param p	configuration of the parent
    	 * @param c	control, part of motion to get here
    	 * @param d	duration, part of motion to get here
    	 */
    	public Edge(Vector p, Vector c, double d) {
    		parent = p;
    		control = c;
    		duration = d;
    	}
    	// some getter methods
    	public Vector getParentConfig() {
    		return parent;
    	}
    	
    	public Vector getControl() {
    		return control;
    	}
    	
    	public double getDuration() {
    		return duration;
    	}
    	
    	
    }
    
    /**
     * Constructor 
     * @param environment the workspace
     * @param robot       the robot
     */
    public RRTPlanner(Environment environment, Robot robot, int k, boolean randDelta, boolean goalBias) {
        super(environment, robot);
        this.setup();
        this.k = k;
        this.randDelta = randDelta;  
        this.goalBias = goalBias;
    }
    
    @Override
    public List<Pair<Point2D, Point2D>> getEdges() {
        // YOU WILL WRITE THIS METHOD
    	// list to store pairs
    	List<Pair<Point2D, Point2D>> list = new ArrayList<Pair<Point2D, Point2D>>();
    	// iterate through nodes in tree
    	for (Vector v:parents.keySet()) {
    		// get parent
    		Vector p = parents.get(v).getParentConfig();
    		// only draw edges if parent isn't null
    		if (p!=null) {
    			// create a pair of x,y coordinates
    			Point2D child = new Point2D(v.get(0), v.get(1));
    			Point2D parent = new Point2D(p.get(0), p.get(1));
    			Pair edge = new Pair(parent, child);
    			// add to list
    			list.add(edge);
    		}
    	}
        return list;
    }
    
    @Override
    public int getSize() {
        // YOU WILL WRITE THIS METHOD
    	// return number of nodes in tree
        return parents.size();
    }

    @Override
    protected void setup() {
        // YOU WILL WRITE THIS METHOD
    	// initialize the tree
    	parents = new HashMap<Vector, Edge>();
    	// add start config; there is no parent
    	parents.put(getStart(), new Edge(null, null, 0));
    }
    
    @Override
    protected void growMap(int K) {
        // YOU WILL WRITE THIS METHOD
    	int i=0;
    	while (i<K) {
    		// get random configuration
    		Vector rand = this.getRobot().getRandomConfiguration(this.getEnvironment(), random);
    		// get boolean for goal bias (gives us 1/100 chance)
    		if (goalBias&&random.nextInt(100)==0) {
    			// if both true we set the goal as the configuration
    			rand = this.getGoal();
    		}
    		// get nearest neighbor(s)
    		List<Vector> kNear = nearestKNeighbors(parents.keySet(), rand, k);
    		for (Vector qnear:kNear) {
    			// use newConf to add this nearest neighbor
    			if (randDelta) {
        			// random number between 0.05 and 0.2
        			newConf(qnear, (0.05+0.15*random.nextDouble()));
        		}
        		else {
        			newConf(qnear, DEFAULT_DELTA);
        		}
    		}
    		i++;
    	}
    }
    
    
    /**
     * Generate a new configuration from a configuration and insert it
     * @param qnear    the beginning configuration of the random motion
     * @param duration the duration of the random motion
     * @return true if one new configuration is inserted, and false otherwise
     */
    @SuppressWarnings("boxing")
    private boolean newConf(Vector qnear, double duration) {
        // YOU WILL WRITE THIS METHOD
    	// get a random control
    	Vector control = this.getRobot().getRandomControl(random);
    	// make sure it's collision free
    	Trajectory trajectory = new Trajectory(control, duration);
    	if (this.getEnvironment().isValidMotion(this.getRobot(), qnear, trajectory, RESOLUTION)) {
    		// get the actual new configuration
    		Vector newConfig = this.getRobot().move(qnear, control, duration);
    		// if configuration is not duplicate add to tree
    		if(!parents.containsKey(newConfig)) {
    			parents.put(newConfig, new Edge(qnear, control, duration));
    			return true;
    		}
    	}
        return false;
    }
    /* improved version
     * should reduce calls to isValidMotion and thus
     * isValidConfiguration
     */
//    @SuppressWarnings("boxing")
//    private boolean newConf(Vector qnear, double duration) {
//        // YOU WILL WRITE THIS METHOD
//    	// get a random control
//    	Vector control = this.getRobot().getRandomControl(random);
//    	// make sure it's collision free
//    	Trajectory trajectory = new Trajectory(control, duration);
//    	// get the actual new configuration
//		Vector newConfig = this.getRobot().move(qnear, control, duration);
//		// if configuration is not duplicate add to tree
//		if(!parents.containsKey(newConfig)) {
//			if (this.getEnvironment().isValidMotion(this.getRobot(), qnear, trajectory, RESOLUTION)) {
//    			parents.put(newConfig, new Edge(qnear, control, duration));
//    			return true;
//    		}
//    	}
//        return false;
//    }
    
    @SuppressWarnings("boxing")
    @Override
    protected Trajectory findPath() {
        // YOU WILL WRITE THIS METHOD
    	// create trajectory from nearest to goal to start
    	Trajectory trajectory = new Trajectory();
    	// List to store individual trajectories, to be reversed later
    	ArrayList<Trajectory> temp = new ArrayList<Trajectory>();
    	// find nearest neighbor to the goal configuration
    	Vector nearest = nearestNeighbor(parents.keySet(), getGoal());
    	// get edge
    	Edge currEdge = parents.get(nearest);
    	// keep on appending until we reach the start
    	while (currEdge.getParentConfig()!=null) {
    		temp.add(new Trajectory(currEdge.getControl(),currEdge.getDuration()));
    		currEdge = parents.get(currEdge.getParentConfig());
    	}
    	// reverse list
    	Collections.reverse(temp);
    	// add trajectories in list to an overall trajectory
    	for (Trajectory traj:temp) {
    		trajectory.append(traj);
    	}
        return trajectory;
    }

    @Override
    protected void reset() {
        // YOU WILL WRITE THIS METHOD
    	// clear tree
    	parents.clear();
    }

}
