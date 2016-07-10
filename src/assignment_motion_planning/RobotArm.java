package assignment_motion_planning;

import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Area;
import java.awt.geom.Path2D;
import java.awt.geom.PathIterator;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

public class RobotArm extends Robot {
    private static final double WIDTH = (1 + Math.sqrt(5.0)) / 2.0;
    private static final double HEIGHT = 0.3;
    private List<Shape> shapes;
    
    private static final Path2D.Double RECTANGLE = new Path2D.Double();
    static {
        double halfHeight = HEIGHT / 2.0;
        RECTANGLE.moveTo(0, halfHeight);
        RECTANGLE.lineTo(0, -halfHeight);
        RECTANGLE.lineTo(WIDTH, -halfHeight);
        RECTANGLE.lineTo(WIDTH, halfHeight);
        RECTANGLE.closePath();
    }
    
    /**
     * Constructor
     * @param s  shapes
     */
    private RobotArm(Shape...s) {
        this.shapes = Collections.unmodifiableList(Arrays.asList(s));
    }
    
    /**
     * Get an robot arm with K links
     * @param K the number of links
     * @return an robot arm with K links
     */
    public static RobotArm getRobotArm(int K) {
        Shape[] shapes = new Shape[K];
        for (int i = 0; i < K; ++i)
            shapes[i] = new Path2D.Double(RECTANGLE);
        return new RobotArm(shapes);
    }
    
    @Override
    public List<Vector> getTransformation(Vector configuration) {
        assert(configuration.getDimension() == getDimension());
        Vector[] result = new Vector[getDimension()];
        double x = 0.0, y = 0.0, theta = normalize(configuration.get(0));
        result[0] = new Vector(x, y, theta);
        for (int i = 1; i < configuration.getDimension(); ++i) {
            x += WIDTH * Math.cos(theta);
            y += WIDTH * Math.sin(theta);
            theta = normalize(theta + configuration.get(i));
            result[i] = new Vector(x, y, theta);
        }
        return Arrays.asList(result);
    }
    
    @Override
    public Area getArea(Vector configuration) {
        assert(configuration.getDimension() == getDimension());
        List<Vector> transforms = getTransformation(configuration);
        Area result = new Area();
        AffineTransform trans = new AffineTransform();
        for (int i = 0; i < getDimension(); ++i) {
            Vector t = transforms.get(i);
            trans.translate(t.get(0), t.get(1));
            trans.rotate(t.get(2));
            result.add(new Area(trans.createTransformedShape(shapes.get(i))));
            trans.setToIdentity();
        }
        return result;
    }
    
    @Override
    public List<PathIterator> getShape() {
        return shapes.stream().map(o -> o.getPathIterator(null)).collect(Collectors.toList());
    }
    
    @Override
    public int getDimension() {
        return shapes.size();
    }

    @Override
    public Vector getRandomConfiguration(Environment environment, Random random) {
        double[] angles = new double[getDimension()];
        for (int i = 0; i < getDimension(); ++i)
            angles[i] = random.nextDouble() * Math.PI * 2.0;
        return new Vector(angles);
    }

    @Override
    public double getMetric(Vector q1, Vector q2) {
        assert(q1.getDimension() == getDimension());
        assert(q2.getDimension() == getDimension());
        double result = 0.0;
        for (int i = 0; i < getDimension(); ++i) {
            result += Robot.getAbsDiff(q1.get(i), q2.get(i));
        }
        return result;
    }

    @Override
    public Vector getRandomControl(Random random) {
        double[] control = new double[getDimension()];
        for (int i = 0; i < getDimension(); ++i)
            control[i] = random.nextDouble() * 2.0 - 1.0;
        return new Vector(control);
    }

    @Override
    public Vector move(Vector configuration, Vector control, double duration) {
        assert(configuration.getDimension() == getDimension());
        assert(control.getDimension() == getDimension());
        double[] config = new double[getDimension()];
        for (int i = 0; i < getDimension(); ++i) {
            config[i] = normalize(configuration.get(i) + control.get(i) * duration);
        }
        return new Vector(config);
    }

    @Override
    public Vector normalize(Vector configuration) {
        assert(configuration.getDimension() == getDimension());
        double[] config = new double[getDimension()];
        for (int i = 0; i < getDimension(); ++i)
            config[i] = normalize(configuration.get(i));
        return new Vector(config);
    }
    
    /**
     * Steering method for the robot arm
     */
    @Override
    protected Trajectory steer(Vector q1, Vector q2) {
        assert(q1.getDimension() == getDimension());
        assert(q2.getDimension() == getDimension());
        assert(!q1.equals(q2));
        double[] differences = new double[getDimension()];
        double[] control = new double[getDimension()];
        double duration = 0.0;
        for (int i = 0; i < getDimension(); ++i) {
            double difference = Robot.getSignediff(q1.get(i), q2.get(i));
            control[i] = difference >= 0.0 ? 1.0 : -1.0;
            differences[i] = Math.abs(difference);
            if (duration < differences[i])
                duration = differences[i];
        }
        for (int i = 0; i < getDimension(); ++i) {
            control[i] *= differences[i] / duration;
        }
        return new Trajectory(new Vector(control), duration);
    }
}
