/**
 * This algorithm is based on the following paper:
 * Lydia E. Kavraki, Petr Svestka, Jean-Claude Latombe, and Mark H. Overmars,
 * Probabilistic roadmaps for path planning in high-dimensional configuration spaces, 
 * IEEE Transactions on Robotics and Automation 12 (4): 566–580.
 * http://dx.doi.org/10.1109/70.508439
 */
package assignment_motion_planning;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.NavigableSet;
import java.util.Random;
import java.util.Set;
import java.util.TreeSet;

public class PRMPlanner extends MotionPlanner {
    private int numberOfAttempts = 10;
    // map that acts as the graph for the roadmap
    private HashMap<Vector, HashMap<Vector, Double>> graph;
    
    /**
     * Constructor
     * @param environment  the workspace
     * @param robot        a robot with a steering method
     */
    public PRMPlanner(Environment environment, Robot robot) {
        super(environment, robot);
        // call setup() to initialize graph
        this.setup();
    }
    
    @Override
    public int getSize() {
        // YOU WILL WRITE THIS METHOD
    	// return number of nodes in graph
    	return graph.size();
    }

    @Override
    protected void setup() {
        // YOU WILL WRITE THIS METHOD
    	// initialize graph
        graph = new HashMap<Vector, HashMap<Vector, Double>>();
        // add start and goal into graph
        graph.put(getStart(), new HashMap<Vector, Double>());
        graph.put(getGoal(), new HashMap<Vector, Double>());
    	
    }

    @Override
    protected void growMap(int K) {
        // YOU WILL WRITE THIS METHOD
    	// generate free configs and add them for K times
    	int i=0;
    	while (i<K) {
    		// generate free configuration
    		Vector v = generateFreeConfiguration();
    		// add free config as a vertex if not null
    		if (v!=null) {
    			addVertex(v);
    			i++;
    		}
    	}
    }
    
    /**
     * Add a free configuration to the roadmap
     * @param free  a free configuration
     */
    @SuppressWarnings("boxing")
    private void addVertex(Vector free) {
        // YOU WILL WRITE THIS METHOD
    	// get the K nearest neighbors and loop through them
    	List<Vector> neighbors = nearestKNeighbors(graph.keySet(), free, this.kValue());
    	for (Vector neighbor:neighbors) {
    		// check that the new config is not the same as the neighbor
    		if (!free.equals(neighbor)) {
    			// check a collision-free path exists from neighbor to free
    			if (this.getEnvironment().isSteerable(this.getRobot(), neighbor, free, RESOLUTION)){
    				// get the metric
    				double metric = this.getRobot().getMetric(free, neighbor);
    				// add new config as vertex reachable by neighbor
    				graph.get(neighbor).put(free, metric);
    			}
    			// check a collision-free path exists from new config to neighbor
    			if (this.getEnvironment().isSteerable(this.getRobot(), free, neighbor, RESOLUTION)){
    				// get the metric
    				double metric = this.getRobot().getMetric(neighbor, free);
    				// add neighbor as vertex reachable by new config
    				// if new config is not yet in the graph, add it
    				if (!graph.containsKey(free)) {
    					graph.put(free, new HashMap<Vector, Double>());
    					graph.get(free).put(neighbor, metric);
    				}
    				// else new config is already in graph, add the edge
    				else {
    					graph.get(free).put(neighbor, metric);
    				}
    			}
    		}
    	}
    }
    
    /* improved version of addVertex
     * cuts the number of calls to isSteerable() and thus
     * isValidPath, isValidConfiguration, and getArea by 
     * approximately half
     */
//    @SuppressWarnings("boxing")
//    private void addVertex(Vector free) {
//        // YOU WILL WRITE THIS METHOD
//    	// get the K nearest neighbors and loop through them
//    	List<Vector> neighbors = nearestKNeighbors(graph.keySet(), free, this.kValue());
//    	for (Vector neighbor:neighbors) {
//    		// check that the new config is not the same as the neighbor
//    		if (!free.equals(neighbor)) {
//    			// check a collision-free path exists from neighbor to free
//    			if (this.getEnvironment().isSteerable(this.getRobot(), neighbor, free, RESOLUTION)){
//    				// get the metric
//    				double metric = this.getRobot().getMetric(free, neighbor);
//    				// add new config as vertex reachable by neighbor
//    				graph.get(neighbor).put(free, metric);
//    				// add neighbor as reachable by new config
//    				if (!graph.containsKey(free)) {
//    					graph.put(free, new HashMap<Vector, Double>());
//    					graph.get(free).put(neighbor, metric);
//    				}
//    				// else new config is already in graph, add the edge
//    				else {
//    					graph.get(free).put(neighbor, metric);
//    				}
//    			}
//    		}
//    	}
//    }

    @Override
    protected void reset() {
        // YOU WILL WRITE THIS METHOD
    	// clear the data structure
    	graph.clear();
    }   
    
    /**
     * 
     */
    @Override
    protected Trajectory query() {
        addVertex(getStart());
        addVertex(getGoal());
        return findPath();
    }
    
    /**
     * Generate a free configuration
     * @return a free configuration if possible, and null otherwise
     */
    private Vector generateFreeConfiguration() {
        // YOU WILL WRITE THIS METHOD
    	// int to keep track of how many attempts we have had
    	// we go no more than numberOfAttempts attempts
    	int attempts = 0;
    	while (attempts<numberOfAttempts) {
    		// get a random configuration for robot in environment
    		Vector rand = this.getRobot().getRandomConfiguration(this.getEnvironment(), random);
    		// if configuration is collision free, return it
    		if (this.getEnvironment().isValidConfiguration(this.getRobot(), rand)){
    			return rand;
    		}
    		// increment num of attempts
    		attempts++;
    	}
    	// if we have reached max num of attempts and still not found a valid config
    	// we return null since we have found nothing
        return null;
    }
    
    /**
     * The number of vertices for a new vertex attempts to connect.
     * The following paper describes a way to determine the value in order
     * to ensure asymptotic optimality.
     * Sertac Karaman and Emilio Frazzoli, 
     * Sampling-based Algorithms for Optimal Motion Planning, 
     * International Journal of Robotics Research, vol. 30, no.7, pp. 846-894, 2011.
     * http://dx.doi.org/10.1177/0278364911406761
     * @return number of neighbors for a new vertex attempts to connect.
     */
    private int kValue() {
        return 15; // Magic number suggested in Steven M. LaValle's "Planning Algorithms"
    }
    
    /**
     * Determine whether this edge connecting two configurations can be ignored.
     * The following paper describes a way to ignore some edges while maintaining
     * asymptotic optimality.
     * Weifu Wang, Devin J. Balkcom, and Amit Chakrabarti
     * A fast online spanner for roadmap construction
     * International Journal of Robotics Research, vol. 34, no.11, pp. 1418-1432, 2015.
     * http://dx.doi.org/10.1177/0278364915576491
     * @param u first configuration
     * @param v second configuration
     * @return true if this edge can be ignored, and false otherwise. 
     */
    private boolean safeToIgnore(Vector u, Vector v) {
        return false; // Extra credit!!
    }
    
    @Override
    protected Trajectory findPath() {
        List<Vector> path = aStar(getStart(), getGoal());
        return path != null ? convertToTrajectory(path) : null; 
    }
    
    /**
     * Convert a list of configurations to a corresponding trajectory based on the steering method
     * @param path a list of configurations
     * @return a trajectory
     */
    private Trajectory convertToTrajectory(List<Vector> path) {
        Trajectory result = new Trajectory();
        Vector previous = path.get(0);
        for (int i = 1; i < path.size(); ++i) {
            Vector next = path.get(i);
            result.append(getRobot().steer(previous, next));
            previous = next;
        }
        return result;
    }
    
    /**
     * Astar search
     * @return a path
     */
    // this is the A* search that is provided as part of the assignment
    @SuppressWarnings("boxing")
    private List<Vector> aStar(Vector start, Vector goal) {
    	
    	System.out.println("Start "+start);
    	System.out.println("Goal "+goal);
    	
        NavigableSet<Node> pq = new TreeSet<>();
        Map<Vector, Node> map = new HashMap<>();
        Node root = new Node(start, null, 0, getRobot().getMetric(start, goal));
        pq.add(root);
        map.put(start, root);
        while (!pq.isEmpty()) {
            Node node = pq.pollFirst();
            Vector configuration = node.getConfiguration();
            double cost = node.getCost();
            if (goal.equals(configuration)) {
                return backChain(node);
            }
            for (Vector config : getSuccessors(configuration)){
                double heuristic = getRobot().getMetric(config, goal);
                // new cost should be sum of current cost and weight of edge from current
                // node to the config in successors
                double newCost = graph.get(configuration).get(config)+cost; // YOU NEED TO MODIFY THIS LINE
                Node test = map.get(config);
                
                if (test != null) {
                    if (test.getPriority() > newCost + heuristic)
                        pq.remove(test);
                    else
                        continue;
                }
                Node newNode = new Node(config, node, newCost, heuristic);
                map.put(config, newNode);
                pq.add(newNode);
            }
        }
        return null;
    }
    
    /**
     * Get successors for a configuration
     * @param configuration
     * @return a collection of successors
     */
    private Collection<Vector> getSuccessors(Vector configuration) {
        // YOU WILL WRITE THIS METHOD
    	// we return the vectors to which the config is connected
    	// need to make sure config is actually in the graph
    	if (graph.containsKey(configuration))
    		return graph.get(configuration).keySet();
    	return new HashSet<Vector>();
    }
    
    /**
     * Backchain to construct a path
     * @param node the end node
     * @return a path
     */
    private static List<Vector> backChain(Node node) {
        LinkedList<Vector> result = new LinkedList<>();
        for (Node current = node; current != null; current = current.getParent()) {
            result.addFirst(current.getConfiguration());
        }
        return result;
    }
    
    final class Node implements Comparable<Node> {
        private Vector configuration;
        private Node parent;
        private double heuristic;
        private double cost;
        
        public Node(Vector config, Node p, double cost, double heuristic) {
            this.configuration = config;
            this.parent = p;
            this.heuristic = heuristic;
            this.cost = cost;
        }
        
        @Override
        public int compareTo(Node o) {
            int comparison = Double.compare(getPriority(), o.getPriority());
            return comparison == 0 ? configuration.compareTo(o.getConfiguration()) : comparison;
        }
        
        /**
         * Get the parent
         * @return the parent
         */
        public Node getParent(){
            return parent;
        }
        
        /**
         * Get the configuration
         * @return the configuration
         */
        public Vector getConfiguration() {
            return configuration;
        }
        
        /**
         * Get the cost
         * @return the cost
         */
        public double getCost() {
            return cost;
        }
        
        /**
         * Get the heuristic
         * @return the heuristic
         */
        public double getHeuristic() {
            return heuristic;
        }
        
        /**
         * Get the priority 
         * @return the priority
         */
        public double getPriority(){
            return getCost() + getHeuristic();
        }
    }
}
