package assignment_motion_planning;

import java.util.ArrayList;
import java.util.List;

public class Trajectory {
    private List<Vector> controls = new ArrayList<>();
    private List<Double> durations = new ArrayList<>();
    
    /**
     * Constructor
     */
    public Trajectory() {}
    
    /**
     * Constructor
     * @param control   the initial control
     * @param duration  the duration
     */
    public Trajectory(Vector control, double duration) {
        addControl(control, duration);
    }
    
    /**
     * Get the index-th control
     * @param index the index of the controls
     * @return the index-th control
     */
    public Vector getControl(int index) {
        return controls.get(index);
    }
    
    /**
     * Get the index-th duration
     * @param index the index of the controls
     * @return the index-th control
     */
    @SuppressWarnings("boxing")
    public double getDuration(int index) {
        return durations.get(index);
    }
    
    /**
     * Get the number of actions in the trajectory
     * @return the number of actions in the trajectory
     */
    public int size() {
        return controls.size();
    }
    
    /**
     * Append a trajectory to this trajectory
     * @param trajectory a trajectory
     */
    public void append(Trajectory trajectory) {
        for (int i = 0; i < trajectory.size(); ++i) {
            addControl(trajectory.getControl(i), trajectory.getDuration(i));
        }
    }
    
    /**
     * @param control   a control
     * @param duration  corresponding duration
     */
    @SuppressWarnings("boxing")
    public void addControl(Vector control, double duration) {
        assert(control != null);
        assert(duration > 0);
        controls.add(control);
        durations.add(duration);
        
    }
    
    /**
     * Compute the total duration
     * Note: the sum function implements Kahan summation algorithm
     * @return the total duration
     */
    @SuppressWarnings("boxing")
    public double totalTime() {
        return durations.stream().mapToDouble(t -> t).sorted().sum();
    }
    
    
    /**
     * String representation of the trajectory
     */
    @Override
    public String toString() {
        StringBuilder result = new StringBuilder("Trajectory:");
        for (int i = 0; i < controls.size(); ++i) {
            result.append(String.format(" (%s, %.2f)", controls.get(i).toString(), durations.get(i)));
        }
        return result.toString();
    }
}
