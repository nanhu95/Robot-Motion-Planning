package assignment_motion_planning;

import java.awt.Point;
import java.awt.Rectangle;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Area;
import java.awt.geom.Path2D;
import java.awt.geom.PathIterator;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;

public class PlanarRobot extends Robot {
    private static final int DIMENSION = 3;
    private static final double GOLDEN_RATIO = (1 + Math.sqrt(5.0)) / 2.0;
    private static final double NORMAL_SCALE = 0.2;
    private static final double ANT_MAN_SCALE = 0.01;
    private static final boolean IS_ANT_MAN = false;
    private static final double WEIGHT_FOR_ANGLE = 1.0;
    private Shape shape;
    private List<Vector> controls;
    
    /**
     * Constructor
     * @param s  shape of the robot
     * @param us control set
     */
    private PlanarRobot(Shape s, Vector... us) {
        this.shape = s;
        controls = Collections.unmodifiableList(Arrays.asList(us));
    }
    
    /**
     * Get a Dubins car
     * @return a Dubins car
     */
    public static PlanarRobot getDubinsCar() {
        return DUBINS_CAR;
    }
    
    /**
     * Get a Reeds-Shepp car
     * @return a Reeds-Shepp car
     */
    public static PlanarRobot getReedsSheppCar() {
        return REEDS_SHEPP_CAR;
    }
    
    /**
     * Get a differential drive
     * @return a differential drive
     */
    public static PlanarRobot getDifferentialDrive() {
        return DIFFERENTIAL_DRIVE;
    }
    
    /**
     * Get an omnidirectional robot
     * @return an omnidirectional robot
     */
    public static PlanarRobot getOmnidirectionalRobot() {
        return OMNI;
    }
    
    @Override
    public List<Vector> getControls() {
        return controls;
    }
    
    @Override
    public List<PathIterator> getShape() {
        return Collections.singletonList(shape.getPathIterator(null));
    }
    
    @Override
    public List<Vector> getTransformation(Vector configuration) {
        assert(configuration != null);
        return Collections.singletonList(configuration);
    }
    
    @Override
    public int getDimension() {
        return DIMENSION;
    }
    
    @Override
    public double getMetric(Vector q1, Vector q2) {
        assert(q1.getDimension() == getDimension());
        assert(q2.getDimension() == getDimension());
        // Math.hypot is slower but more accurate than Math.sqrt
        double distance = Math.hypot(q1.get(0) - q2.get(0), q1.get(1) - q2.get(1));
        return Math.max(distance, WEIGHT_FOR_ANGLE * Robot.getAbsDiff(q1.get(2), q2.get(2)));
    }
    
    @Override
    public Vector getRandomConfiguration(Environment environment, Random random) {
        Rectangle bound = environment.getBounds();
        Point p = bound.getLocation();
        double width = bound.getWidth(), height = bound.getHeight();
        double x = random.nextDouble() * width + p.getX();
        double y = random.nextDouble() * height + p.getY();
        double theta = random.nextDouble() * Math.PI * 2.0;
        return new Vector(x, y, theta);
    }
    
    @Override
    public Vector getRandomControl(Random random) {
        return controls.get(random.nextInt(controls.size()));
    }

    @Override
    public Area getArea(Vector configuration) {
        assert(configuration.getDimension() == getDimension());
        AffineTransform trans = new AffineTransform();
        trans.translate(configuration.get(0), configuration.get(1));
        trans.rotate(configuration.get(2));
        return new Area(trans.createTransformedShape(shape));
    }

    @Override
    public Vector move(Vector configuration, Vector control, double duration) {
        assert(configuration.getDimension() == getDimension());
        assert(control.getDimension() == getDimension());
        return new Transformation(configuration).move(control, duration).toConfiguration();
    }
    
    private static final Vector[] DUBINS_CONTROLS = new Vector[] { new Vector(1, 0, 0), 
                                                                   new Vector(1, 0, 1),
                                                                   new Vector(1, 0, -1)};
    private static final Vector[] RS_CONTROLS = new Vector[] { new Vector(1, 0, 0), 
                                                               new Vector(1, 0, 1), 
                                                               new Vector(1, 0, -1),
                                                               new Vector(-1, 0, 0),
                                                               new Vector(-1, 0, 1),
                                                               new Vector(-1, 0, -1)};
            
    private static final Vector[] DD_CONTROLS = new Vector[] { new Vector(1, 0, 0), 
                                                               new Vector(-1, 0, 0),
                                                               new Vector(0, 0, 1), 
                                                               new Vector(0, 0, -1)};
    private static final double c = Math.sqrt(3.0) / 3.0;
    private static final double c2 = Math.sqrt(3.0) / 1.5;
    private static final Vector[] OD_CONTROLS = new Vector[] { new Vector(c, 1.0, 0.0),
                                                               new Vector(-c, -1.0, 0.0),
                                                               new Vector(-c, 1.0, 0.0),
                                                               new Vector(c, -1.0, 0.0),
                                                               new Vector(-c2, 0.0, 0.0),
                                                               new Vector(c2, 0.0, 0.0),
                                                               new Vector(0.0, 0.0, 1.0),
                                                               new Vector(0.0, 0.0, -1.0),
                                                               new Vector(0.0, -4.0/3.0, 1.0/3.0),
                                                               new Vector(0.0, 4.0/3.0, -1.0/3.0),
                                                               new Vector(c2, 2.0/3.0, 1.0/3.0),
                                                               new Vector(-c2, -2.0/3.0, -1.0/3.0),
                                                               new Vector(-c2, 2.0/3.0, 1.0/3.0),
                                                               new Vector(c2, -2.0/3.0, -1.0/3.0)};
    private static final Path2D.Double RECTANGLE = new Path2D.Double();
    static {
        double scale = IS_ANT_MAN ? ANT_MAN_SCALE : NORMAL_SCALE;
        double halfWidth = scale * GOLDEN_RATIO;
        double halfHeight = scale;
        RECTANGLE.moveTo(-halfWidth, halfHeight);
        RECTANGLE.lineTo(-halfWidth, -halfHeight);
        RECTANGLE.lineTo(halfWidth, -halfHeight);
        RECTANGLE.lineTo(halfWidth, halfHeight);
        RECTANGLE.closePath();
    }
    
    private static final Path2D.Double TRIANGLE = new Path2D.Double();
    static {
        double scale = IS_ANT_MAN ? ANT_MAN_SCALE : NORMAL_SCALE;
        double halfWidth = scale * GOLDEN_RATIO;
        double x = halfWidth / (2.0 * Math.sqrt(3.0));
        double y = halfWidth / (2.0);
        TRIANGLE.moveTo(halfWidth / Math.sqrt(3.0), 0);
        TRIANGLE.lineTo(-x, y);
        TRIANGLE.lineTo(-x, -y);
        TRIANGLE.closePath();
    }
    
    private static final PlanarRobot DUBINS_CAR 
        = new PlanarRobot(RECTANGLE, DUBINS_CONTROLS);
    private static final PlanarRobot REEDS_SHEPP_CAR 
        = new PlanarRobot(RECTANGLE, RS_CONTROLS);
    private static final PlanarRobot DIFFERENTIAL_DRIVE 
        = new PlanarRobot(RECTANGLE, DD_CONTROLS);
    private static final PlanarRobot OMNI 
        = new PlanarRobot(TRIANGLE, OD_CONTROLS);
    
    private static final class Transformation {
        private final double x;
        private final double y;
        private final double sin;
        private final double cos;
        private static final double EPSILON = 1e-7;
        
        /**
         * Construct from a configuration
         * @param configuration a configuration
         */
        public Transformation(Vector configuration) {
            assert(configuration != null);
            assert(configuration.getDimension() == 3);
            x = configuration.get(0);
            y = configuration.get(1);
            double theta = configuration.get(2);
            sin = Math.sin(theta);
            cos = Math.cos(theta);
        }
        
        /**
         * Construct from position and orientation
         * @param arg_x x-coordinate
         * @param arg_y y-coordinate
         * @param arg_sin sin theta
         * @param arg_cos cos theta
         */
        public Transformation(double arg_x, double arg_y, double arg_sin, double arg_cos) {
            x = arg_x;
            y = arg_y;
            sin = arg_sin;
            cos = arg_cos;
        }
        
        /**
         * Create a transformation from a control and its duration
         * @param control control
         * @param time duration
         */
        public Transformation(Vector control, double time) {
            double vx = control.get(0), vy = control.get(1), omega = control.get(2);
            double theta = omega * time, sinc = sinc(theta), verc = verc(theta);
            x = time * (vx * sinc - vy * verc);
            y = time * (vx * verc + vy * sinc);
            sin = Math.sin(theta);
            cos = Math.cos(theta);
        }
        
        /**
         * Convert a transformation to a configuration
         * @return a configuration represented by the transformation
         */
        public Vector toConfiguration() {
            return new Vector(getX(), getY(), normalize(getTheta()));
        }
        
        /**
         * @param T the transformation to be transformed
         * @return a combined transformation
         */
        public Transformation transform(Transformation T) {
            return new Transformation(cos * T.getX() - sin * T.getY() + x,
                                      sin * T.getX() + cos * T.getY() + y,
                                      sin * T.getCos() + cos * T.getSin(),
                                      cos * T.getCos() - sin * T.getSin());
        }
        
        /**
         * Integrate a control with a duration
         * @param control: control want to apply
         * @param time: duration of the control
         * @return a new Transformation of the final configuration
         */
        public Transformation move(Vector control, double time) {
            return this.transform(new Transformation(control, time));
        }
        
        /**
         * @return x-coordinate
         */
        public double getX() {
            return x;
        }
        
        /**
         * @return y-coordinate
         */
        public double getY() {
            return y;
        }
        
        /**
         * @return sin value
         */
        public double getSin() {
            return sin;
        }
        
        /**
         * @return cos value
         */
        public double getCos() {
            return cos;
        }
        
        /**
         * @return theta
         */
        public double getTheta() {
            return Math.atan2(getSin(), getCos());
        }
        
        /**
         * @param theta
         * @return sinc value
         */
        private static double sinc(double theta) {
            if (Math.abs(theta) < EPSILON) 
                return 1.0;
            return Math.sin(theta) / theta;
        }
        
        /**
         * @param theta
         * @return verc value
         */
        private static double verc(double theta) {
            if (Math.abs(theta) < EPSILON) 
                return 0.0;
            return (1.0 - Math.cos(theta)) / theta;
        }
        
        @SuppressWarnings("boxing")
        @Override
        public String toString() {
            return String.format("Transformation: (%.2f, %.2f, %.2f, %.2f)", getX(), getY(), getCos(), getSin());
        }
    }
}
