package assignment_motion_planning;

import java.awt.geom.PathIterator;
import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

import javafx.animation.KeyFrame;
import javafx.animation.Timeline;
import javafx.animation.TranslateTransition;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.geometry.Point2D;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.transform.Rotate;
import javafx.scene.shape.Rectangle;
import javafx.scene.shape.Polygon;
import javafx.scene.shape.Shape;
import javafx.scene.shape.Line;
import javafx.stage.Stage;
import javafx.util.Duration;
import javafx.util.Pair;
import javafx.scene.paint.Color;
import javafx.animation.SequentialTransition;
import javafx.animation.ParallelTransition;
import javafx.animation.TranslateTransition;
import javafx.animation.Animation;
import javafx.animation.KeyValue;

import javax.imageio.ImageIO;

import javafx.embed.swing.SwingFXUtils;
import javafx.scene.layout.StackPane;

public class MotionPlannerView extends Group {
    private static final int SCALE = 80;
    private static final boolean SAVE_IMAGES = false;
    private List<Shape> robotShapes;
    private Duration duration = Duration.millis(300);  // speed of animation
    private Robot robot;
    private double height;
    private double width;
    private StackPane root;
    private int currentMove;
    
    /**
     * Constructor
     * @param primaryStage the stage
     * @param environment  the workspace
     * @param robot        the robot
     */
    public MotionPlannerView(Stage primaryStage, StackPane rot, Environment environment, Robot r) {
        this.robot = r;
        this.root = rot;
        java.awt.Rectangle bounds = environment.getBounds();
        height = bounds.getHeight() * SCALE;
        width = bounds.getWidth() * SCALE;
        primaryStage.setHeight(height);
        primaryStage.setWidth(width);
        for (PathIterator it : environment.getObstacles()) {
            Shape s = converToShape(it);
            s.setFill(Color.BLACK);
            this.getChildren().add(s);    
        }
        robotShapes = new ArrayList<>();
        for (PathIterator it : robot.getShape()) {
            Shape s = converToShape(it);
            robotShapes.add(s);
            s.setFill(Color.GREEN);
            this.getChildren().add(s); 
        }
        
        this.setScaleX(SCALE);
        this.setScaleY(-SCALE);
    }
    
    /**
     * Draw edges
     * @param edges the edges to be drawn
     */
    public void drawEdges(List<Pair<Point2D, Point2D>> edges) {
        for (Pair<Point2D, Point2D> entry : edges) {
            Point2D from = entry.getKey();
            Point2D to = entry.getValue();
            Line line = new Line(from.getX(), from.getY(), to.getX(), to.getY());
            line.setStrokeWidth(1.0 / SCALE);
            this.getChildren().add(line);
        }
    }
    
    /**
     * Animate the path
     * @param path the desired list of configurations of the robot
     */
    public void animate(List<Vector> path) {
        Vector start = path.get(0);
        List<Vector> transform = robot.getTransformation(start);
        for (int i = 0; i < transform.size(); ++i) {
            Shape shape = robotShapes.get(i);
            double x = transform.get(i).get(0);
            double y = transform.get(i).get(1);
            double theta = transform.get(i).get(2);
            shape.setTranslateX(x);
            shape.setTranslateY(y);
            shape.getTransforms().add(new Rotate(Math.toDegrees(theta), 0.0, 0.0));
            
        }
        Animation animation = getAnimation(path);
        animation.setDelay(Duration.ZERO);
        animation.setCycleCount(Animation.INDEFINITE);
        animation.setAutoReverse(false);
        animation.playFromStart();
    }
    
    /**
     * Generate an animation for the path
     * @param path the desired list of configurations of the robot
     * @return a sequential transition that animates the robot moving along the path
     */
    private Animation getAnimation(List<Vector> path) {
        SequentialTransition st = new SequentialTransition();
        Vector previous = path.get(0);
        for (int i = 1; i < path.size(); ++i) {
            Vector next = path.get(i);
            st.getChildren().add(getAnimation(previous, next));
            previous = next;
        }
        st.setDelay(Duration.ZERO);
        return st;
    }
    
    /**
     * Generate an animation for a specific configuration
     * @param previous the previous configuration
     * @param next     the next configuration
     * @return a parallel transition that animates the robot
     */
    private Animation getAnimation(Vector previous, Vector next) {
        ParallelTransition pt = new ParallelTransition();
        pt.setDelay(Duration.ZERO);
        List<Vector> before = robot.getTransformation(previous);
        List<Vector> after = robot.getTransformation(next);
        for (int i = 0; i < robotShapes.size(); ++i) {
            Shape shape = robotShapes.get(i);
            double dx = after.get(i).get(0) - before.get(i).get(0);
            double dy = after.get(i).get(1) - before.get(i).get(1);
            double dtheta = Robot.getSignediff(before.get(i).get(2), after.get(i).get(2));
            dtheta = Math.toDegrees(dtheta);
            pt.getChildren().addAll(getAnimation(shape, dx, dy, dtheta));
        }
        if (SAVE_IMAGES) {
            String filename = String.format("images/%05d.png", ++currentMove);
            Animation rt = new Timeline(new KeyFrame(Duration.ZERO, null), 
                                        new KeyFrame(duration, e -> {
                                            try {
                                                ImageIO.write( SwingFXUtils.fromFXImage( root.snapshot(null, null), null),"png", new File(filename));
                                            } catch (Exception e1) {
                                                e1.printStackTrace();
                                            }
                                    }, null));
            pt.getChildren().add(rt);
        }
        return pt;
    }
    
    /**
     * Generate translation and rotation transitions
     * @param shape  the shape to be moved
     * @param x      the desired x-coordinate
     * @param y      the desired y-coordinate
     * @param theta  the desired angle in degree
     * @return a list of translation and rotation transitions
     */
    private List<Animation> getAnimation(Shape shape, double dx, double dy, double dtheta) {
        TranslateTransition tt = new TranslateTransition(duration, shape);
        tt.setByX(dx);
        tt.setByY(dy);
        tt.setDelay(Duration.ZERO);
        // WHY I cannot specify the rotation center of a RotateTransition?
        //RotateTransition rt = new RotateTransition(duration, shape);
        //rt.setByAngle(dtheta);
        //rt.setDelay(Duration.ZERO);
        //rt.setAxis(Rotate.Z_AXIS);
        Rotate rotate = new Rotate(0.0, 0.0, 0.0);
        shape.getTransforms().add(rotate);
        KeyValue start = new KeyValue(rotate.angleProperty(), 0);
        KeyValue end = new KeyValue(rotate.angleProperty(), dtheta);
        Animation rt = new Timeline(new KeyFrame(Duration.ZERO, start), 
                                    new KeyFrame(duration, end));
        return Arrays.asList(tt, rt);
    }
    
    /**
     * Convert a path iterator to a javafx shape
     * @param it iterator
     * @return a javafx shape
     */
    public Shape converToShape(PathIterator it) {
        double[] coords = new double[6];
        List<Double> points = new ArrayList<>();
        for (; !it.isDone(); it.next()) {
            int type = it.currentSegment(coords);
            if (type == PathIterator.SEG_MOVETO || type == PathIterator.SEG_LINETO) {
                points.add(coords[0]);
                points.add(coords[1]);
            }
        }
        double[] temp = new double[points.size()];
        for (int i = 0; i < points.size(); ++i)
            temp[i] = points.get(i);
        return new Polygon(temp);
    }
}
