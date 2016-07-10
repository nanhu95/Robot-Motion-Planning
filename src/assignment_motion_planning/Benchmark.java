package assignment_motion_planning;

import java.util.List;

public class Benchmark {
    private static final String PLANAR_ROBOT_ENVIRONMENT = "planar_robot_environment";
    private static final String ROBOT_ARM_ENVIRONMENT = "robot_arm_environment";
    private static final int TESTS = 50;
    
    @SuppressWarnings("boxing")
    public static void planarRobot() {
        Environment environment = new Environment(PLANAR_ROBOT_ENVIRONMENT);
        Vector start = new Vector(-4, -4, 0);
        Vector goal = new Vector(4, 4, 0);
        Robot robot = PlanarRobot.getDifferentialDrive();
        double[] error = new double[TESTS];
        double[] rate = new double[TESTS];
        double[] length = new double[TESTS];
        long[] count = new long[TESTS];
        for (int i = 0; i < TESTS; ++i) {
        	// constructor for RRTPlanner has been changed
        	// please see MotionPlannerDriver for details
            MotionPlanner mp = new RRTPlanner(environment, robot, 1, false, false);
            mp.setDefaultSize(100 * (i + 1));
            long beforeCount = environment.getCDCount();
            Trajectory result = mp.solve(start, goal);
            count[i] = environment.getCDCount() - beforeCount;
            List<Vector> path = robot.interpolate(start, result, 0.1);
            Vector finalConfig = path.get(path.size() - 1);
            error[i] = robot.getMetric(goal, finalConfig);
            rate[i] = mp.getFreeSampleRate();
            length[i] = result.totalTime();
            System.out.println(String.format("Error: %f Sample rate: %f Length: %f Collision Detection: %d", 
                    error[i], rate[i], length[i], count[i]));
            
        }
    }
    
    @SuppressWarnings("boxing")
    public static void robotArm() {
        Environment environment = new Environment(ROBOT_ARM_ENVIRONMENT);
        Vector start = new Vector(Math.PI * 1.5, 0.0, -Math.PI / 2.0, 0.0);
        Vector goal = new Vector(0.0, 0.0, Math.PI / 2.0, 0.0);
        Robot robot = RobotArm.getRobotArm(4);
        double[] error = new double[TESTS];
        double[] rate = new double[TESTS];
        double[] length = new double[TESTS];
        long[] count = new long[TESTS];
        MotionPlanner mp = new PRMPlanner(environment, robot);
        mp.setDefaultSize(0);
        mp.solve(start, goal);
        for (int i = 0; i < TESTS; ++i) {
            long beforeCount = environment.getCDCount();
            mp.grow(100);
            Trajectory result = mp.query(start, goal);
            count[i] = environment.getCDCount() - beforeCount;
            List<Vector> path = robot.interpolate(start, result, 0.1);
            Vector finalConfig = path.get(path.size() - 1);
            error[i] = robot.getMetric(goal, finalConfig);
            rate[i] = mp.getFreeSampleRate();
            length[i] = result.totalTime();
            System.out.println(String.format("Error: %f Sample rate: %f Length: %f Collision Detection: %d", 
                    error[i], rate[i], length[i], count[i]));
        }
    }
    
    public static final void main(String[] args) {
        planarRobot();
        //robotArm();
    }
    
}
