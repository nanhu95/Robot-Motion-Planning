package assignment_motion_planning;

import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.AbstractMap.SimpleImmutableEntry;
import java.util.logging.Logger;
import java.util.Collection;
import java.util.stream.Collectors;

import javafx.geometry.Point2D;
import javafx.util.Pair;

public abstract class MotionPlanner {
    private static final Logger logger = Logger.getLogger(MotionPlanner.class.getName());
    protected static final double RESOLUTION = 0.01; // resolution for collision detection
    protected Random random = new Random(0); // Fix the seed to make it easier to debug
    private Robot robot;    // the robot
    private Vector start;   // start configuration
    private Vector goal;    // goal configuration
    private Environment environment;  // workspace
    private int defaultSize = 1000;  // default size of data structures
    private int numberOfRandomSample = 0;
    private int numberOfFreeRandomSample = 0;
    
    /**
     * Constructor
     * @param env the workspace
     */
    protected MotionPlanner(Environment env, Robot r) {
        assert(env != null && r != null);
        this.environment = env;
        this.robot = r;
    }
    
  
    /**
     * Get the edges
     * @return the edges
     */
    public List<Pair<Point2D, Point2D>> getEdges() {
        return null;
    }
    
    /**
     * Find a valid path between startNode and goalNode
     * @param s  the desired start node.
     * @param g  the desired goal node.
     * @return a valid path
     */
    public final Trajectory solve(Vector s, Vector g) {
        assert(s != null && g != null);
        assert(s.getDimension() == getRobot().getDimension());
        assert(g.getDimension() == getRobot().getDimension());
        start = robot.normalize(s);
        goal = robot.normalize(g);
        reset();
        setup();
        grow(defaultSize);
        return findPath();
    }
    
    /**
     * Grow the data structures by a specified amount
     * @param K the number of nodes planner should grow
     */
    @SuppressWarnings("boxing")
    public final void grow(int K) {
        assert(K > 0);
        long startTime = System.currentTimeMillis();
        growMap(K);
        long endTime   = System.currentTimeMillis();
        long totalTime = endTime - startTime;
        logger.info(String.format("Growing %d nodes takes %f seconds. %f nodes per second.", 
                K, totalTime / 1000.0, K * 1000.0 / totalTime));
    }
    
    /**
     * Get the start configuration
     * @return the start configuration
     */
    public final Vector getStart() {
        return start;
    }
    
    /**
     * Get the sample rate
     * @return the ratio
     */
    public double getFreeSampleRate() {
        return numberOfFreeRandomSample / (double) numberOfRandomSample;
    }
    
    /**
     * Increment the number of random samples
     */
    protected void incrementSampleNumber() {
        ++numberOfRandomSample;
    }
    
    /**
     * Increment the number of collision-free random samples
     */
    protected void incrementFreeSampleNumber() {
        ++numberOfFreeRandomSample;
    }
    
    /**
     * Get the goal configuration
     * @return the goal configuration
     */
    public final Vector getGoal() {
        return goal;
    }
    
    /**
     * Get the environment
     * @return the environment
     */
    public final Environment getEnvironment() {
        return environment;
    }
    
    /**
     * Get the robot
     * @return the robot
     */
    public final Robot getRobot() {
        return robot;
    }
    
    /**
     * Initialize the planner before we grow the data structures
     */
    protected abstract void setup();
    
    /**
     * Reset all data structures stored in the planner
     */
    protected abstract void reset();
    
    /**
     * Grow the data structures by a specified amount
     * @param K the number of nodes that the planner grows
     */
    protected abstract void growMap(int K);
    
    /**
     * Find a path.
     * @return a valid path
     */
    protected abstract Trajectory findPath();
    
    /**
     * Get the size (#nodes + #edges) of the data structures used in the planner 
     * @return the size
     */
    public abstract int getSize();
    
    /**
     * Query for a given pair of configurations
     * @param s  the start configuration
     * @param g  the goal configuration
     * @return a valid path
     */
    public final Trajectory query(Vector s, Vector g) {
        assert(s != null && g != null);
        assert(s.getDimension() == getRobot().getDimension());
        assert(g.getDimension() == getRobot().getDimension());
        start = robot.normalize(s);
        goal = robot.normalize(g);
        return query();
    }
    
    /**
     * Implementation for query
     * @return a valid path between start and goal
     */
    protected Trajectory query() {
        return solve(getStart(), getGoal());
    }
    
    /**
     * Change the default size of data structures
     * @param size  the desired size of data structures
     */
    public void setDefaultSize(int size) {
        assert(size > 0);
        this.defaultSize = size;
    }
    
    /**
     * Find query's nearest neighbor in a set of configurations
     * @param configurations  a set of configurations
     * @param query           query configuration
     * @return nearest neighbor
     */
    protected final Vector nearestNeighbor(Collection<Vector> configurations, Vector query) {
        return nearestKNeighbors(configurations, query, 1).get(0);
    }
    
    /**
     * Find query's K-nearest neighbors in a set of configurations
     * @param configurations  a set of configurations
     * @param query           a query configuration
     * @param K               number of neighbors
     * @return K-nearest neighbors
     */
    @SuppressWarnings("boxing")
    protected final List<Vector> nearestKNeighbors(Collection<Vector> configurations, Vector query, int K) {
        assert(query.getDimension() == getRobot().getDimension());
        List<SimpleImmutableEntry<Vector, Double>> temp = configurations.parallelStream()
                                                                        .map(config -> new SimpleImmutableEntry<Vector, Double>(config, getRobot().getMetric(query, config))).collect(Collectors.toList());
        PriorityQueue<SimpleImmutableEntry<Vector, Double>> pq = new PriorityQueue<>(K, (a, b) -> Double.compare(b.getValue(), a.getValue()));
        for (SimpleImmutableEntry<Vector, Double> entry : temp) {
            if (pq.size() == K && entry.getValue() < pq.peek().getValue())
                pq.poll();
            if (pq.size() < K)
                pq.offer(entry);
        }
        return pq.stream().map((n)->n.getKey()).collect(Collectors.toList());
    }
}
