package assignment_motion_planning;

import java.awt.Rectangle;
import java.awt.Shape;
import java.awt.geom.Area;
import java.awt.geom.Path2D;
import java.awt.geom.PathIterator;
import java.io.BufferedReader;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;
import java.util.stream.Collectors;

public class Environment {
    private static final Logger logger = Logger.getLogger(Environment.class.getName());
    private long collisionDetectionCount = 0;
    private Area obstructedArea;
    private List<Shape> obstacles;
    
    /**
     * Read obstacles from a file
     * @param filename the file containing obstacles
     * @throws IOException
     */
    @SuppressWarnings("boxing")
    public Environment(String filename) {        
        Charset charset = Charset.forName("US-ASCII");
        Path file = FileSystems.getDefault().getPath(".", filename);  
        try (BufferedReader br = Files.newBufferedReader(file, charset)) {
            int n = Integer.valueOf(br.readLine());
            obstacles = new ArrayList<>();
            for (int i = 0; i < n; ++i) {
                obstacles.add(readPolygon(br));
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        obstructedArea = new Area();
        for (Shape s : obstacles)
            obstructedArea.add(new Area(s));
        logger.info(String.format("Read %d obstacles", obstacles.size()));
    }
    
    /**
     * Get the number of collision detection checking
     * @return the number of collision detection checking
     */
    public long getCDCount() {
        return collisionDetectionCount;
    }
    
    /**
     * Read a polygon from a BufferedReader
     * @param br input
     * @return a polygon
     * @throws IOException
     */
    @SuppressWarnings("boxing")
    private static Shape readPolygon(BufferedReader br) throws IOException {
        int n = Integer.valueOf(br.readLine());
        Path2D.Double path = new Path2D.Double();
        for (int i = 0; i < n; ++i) {
            String[] line = br.readLine().split(" ");
            double x = Double.valueOf(line[0]), y = Double.valueOf(line[1]);
            if (i == 0)
                path.moveTo(x, y);
            else
                path.lineTo(x, y);
        }
        path.closePath();
        return path;
    }
    
    /**
     * Get path iterators for all obstacles
     * @return a list of PathIterator for all obstacles
     */
    public List<PathIterator> getObstacles() {
        return obstacles.stream().map(o -> o.getPathIterator(null)).collect(Collectors.toList());
    }

    /**
     * Get the bounds of the workspace
     * @return the bounds
     */
    public Rectangle getBounds() {
        return obstructedArea.getBounds();
    }
    
    /**
     * Test whether the steering method leads to a collision free path exists from q1 to q2
     * @param robot       the robot
     * @param q1           begin configuration
     * @param q2           end configuration
     * @param resolution   resolution for collision detection
     * @return true if there is a path, and false otherwise
     */
    public boolean isSteerable(Robot robot, Vector q1, Vector q2, double resolution) {
        assert(robot != null);
        assert(q1.getDimension() == robot.getDimension());
        assert(q2.getDimension() == robot.getDimension());
        return isValidMotion(robot, q1, robot.steer(q1, q2), resolution);
    }
    
    /**
     * Test whether a trajectory is collision free
     * @param robot          the robot
     * @param configuration  the begin configuration
     * @param trajectory     the trajectory
     * @param resolution     the resolution for collision detection
     * @return true if the motion is collision free, and false otherwise.
     */
    public boolean isValidMotion(Robot robot, Vector configuration, Trajectory trajectory,
            double resolution) {
        assert(robot != null);
        assert(configuration.getDimension() == robot.getDimension());
        return isValidPath(robot, robot.interpolate(configuration, trajectory, resolution));
    }
    
    /**
     * Check whether a path is collision-free
     * @param robot  the robot
     * @param path   robot's path
     * @return true if the path is collision-free, and false otherwise
     */
    public boolean isValidPath(Robot robot, List<Vector> path) {
        assert(robot != null);
        return path.parallelStream().allMatch((config)->isValidConfiguration(robot, config));
    }
    
    /**
     * Check whether a configuration is collision-free for a robot
     * @param robot          the robot
     * @param configuration  the configuration
     * @return true if the configuration is collision-free, and false otherwise
     */
    public boolean isValidConfiguration(Robot robot, Vector configuration) {
        assert(robot != null);
        assert(configuration != null);
        ++collisionDetectionCount;
        Area test = (Area)obstructedArea.clone();
        test.intersect(robot.getArea(configuration));
        return test.isEmpty();
    }

}
