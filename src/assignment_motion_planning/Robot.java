package assignment_motion_planning;

import java.awt.geom.Area;
import java.awt.geom.PathIterator;
import java.util.List;
import java.util.Random;
import java.util.ArrayList;

public abstract class Robot {
    
    /**
     * Get the area for a robot at a configuration q
     * @param configuration the configuration
     * @return the area occupied by the robot
     */
    public abstract Area getArea(Vector configuration);
    
    /**
     * Get the shape of the robot
     * @return a list of PathIterator describing the shapes
     */
    public abstract List<PathIterator> getShape();
    
    /**
     * Get the transformation in the global frame for each shape 
     * @return a list of vector representing the transformations
     */
    public abstract List<Vector> getTransformation(Vector configuration);
    
    /**
     * Get the dimension of the configuration of the robot
     * @return the dimension
     */
    public abstract int getDimension();
    
    /**
     * (pseudo)-Metric between two configurations
     * @param q1 the first configuration
     * @param q2 the second configuration
     * @return the (pseudo)-metric
     */
    public abstract double getMetric(Vector q1, Vector q2);
    
    /**
     * Generate a random configuration
     * @param environment  the workspace
     * @param random       a random number generator
     * @return a configuration
     */
    public abstract Vector getRandomConfiguration(Environment environment, Random random);
    
    /**
     * Generate a random control in the control set
     * @param random a random number generator
     * @return a random control
     */
    public abstract Vector getRandomControl(Random random);
    
    /**
     * Apply the control with duration on a robot at given configuration
     * @param configuration  the configuration of the robot
     * @param control        the control to be applied
     * @param duration       the duration
     * @return the resulting configuration
     */
    public abstract Vector move(Vector configuration, Vector control, double duration);
    
    /**
     * Get all possible controls
     * @return a list of controls
     */
    public List<Vector> getControls() {
        return null;
    }
    
    /**
     * Interpolate a trajectory 
     * @param configuration  begin configuration
     * @param trajectory     the trajectory to be interpolated
     * @param resolution     resolution for interpolation
     * @return a list of configurations along the trajectory
     */
    public final List<Vector> interpolate(Vector configuration, Trajectory trajectory, double resolution) {
        assert(configuration.getDimension() == getDimension());
        List<Vector> result = new ArrayList<>();
        result.add(configuration);
        Vector current = configuration;
        for (int i = 0; i < trajectory.size(); ++i) {
            Vector control = trajectory.getControl(i);
            double duration = trajectory.getDuration(i);
            result.addAll(interpolate(current, control, duration, resolution));
            current = move(current, control, duration);
        }
        return result;
    }
    
    /**
     * Interpolate a motion
     * @param configuration  begin configuration
     * @param control        the control to be interpolated
     * @param duration       the duration
     * @param resolution     resolution for interpolation
     * @return a list of configurations along the motion
     */
    public final List<Vector> interpolate(Vector configuration, Vector control, double duration, double resolution) {
        assert(configuration.getDimension() == getDimension());
        List<Vector> result = new ArrayList<>();
        for (double time = resolution; time < duration; time += resolution) {
            result.add(move(configuration, control, time));
        }
        result.add(move(configuration, control, duration));
        return result;
    }
    
    /**
     * Steering method: connecting two configurations without considering obstacles
     * @param q1  begin configuration
     * @param q2  end configuration, which is no equal to begin
     * @return a trajectory
     */
    @SuppressWarnings("static-method")
    protected Trajectory steer(Vector q1, Vector q2) {
        return null;
    }

    /**
     * Normalize the configuration
     * @param configuration the configuration to be normalized
     * @return a normalized configuration
     */
    public Vector normalize(Vector configuration) {
        return configuration;
    }
    
    /**
     * Normalize an angle x to an angle in [0 .. 2pi)
     * @param x an angle
     * @return normalized angle
     */
    public static double normalize(double x) {
        double temp = x % (Math.PI * 2);
        return temp >= 0.0 ? temp : temp + Math.PI * 2;
    }
    
    /**
     * Compute the absolution value of the difference between two angles
     * @param angle1 first angle
     * @param angle2 second angle
     * @return the difference
     */
    public static double getAbsDiff(double angle1, double angle2) {
        double diff = normalize(angle1 - angle2);
        return Math.min((2 * Math.PI) - diff, diff);
    }
    
    /**
     * Compute the signed difference from angle1 to angle2, so that angle1 + signed(angle1, angle2) = anle2
     * @param angle1 first angle
     * @param angle2 second angle
     * @return the difference
     */
    public static double getSignediff(double angle1, double angle2) {
        angle1 = normalize(angle1);
        angle2 = normalize(angle2);
        double forward = angle2 >= angle1 ? angle2 - angle1 : Math.PI * 2.0 - angle1 + angle2;
        double backward = angle2 <= angle1 ? angle1 - angle2 : Math.PI * 2.0 - angle2 + angle1;
        return forward < backward ? forward : -backward;
    }
}
