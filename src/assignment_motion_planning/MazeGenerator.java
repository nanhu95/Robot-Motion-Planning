package assignment_motion_planning;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Random;

public class MazeGenerator {
    
    /**
     * Generate a maze
     * @param numberOfObstacles  number of obstacles to be generated
     * @param xBounds            the bounds of the x-coordinate
     * @param yBounds            the bounds of the y-coordinate
     * @param filename           output filename
     * @throws IOException
     */
    @SuppressWarnings("boxing")
    public static void generate(int numberOfObstacles, double[] xBounds, double[] yBounds, String filename) throws IOException {
        Charset charset = Charset.forName("US-ASCII");
        Path file = FileSystems.getDefault().getPath(".", filename);
        Random random = new Random();
        try (BufferedWriter bw = Files.newBufferedWriter(file, charset)) {
            bw.write(String.format("%d\n", numberOfObstacles + 4));
            double w = xBounds[1] - xBounds[0];
            double h = yBounds[1] - yBounds[0];
            writeRectangle(bw, xBounds[0], yBounds[0], w, 0.1);
            writeRectangle(bw, xBounds[0], yBounds[1], w, 0.1);
            writeRectangle(bw, xBounds[0], yBounds[0], 0.1, h);
            writeRectangle(bw, xBounds[1], yBounds[0], 0.1, h);
            for (int i = 0; i < numberOfObstacles; ++i) {
                double x = random.nextDouble() * (w) + xBounds[0];
                double y = random.nextDouble() * (h) + yBounds[0];
                double width = random.nextDouble() * (xBounds[1] - x) * 0.3;
                double height = random.nextDouble() * (yBounds[1] - y) * 0.3;
                writeRectangle(bw, x, y, width, height);
            }
        }
    }
    
    /**
     * Write a rectangle
     * @param bw     a bufferedwriter
     * @param x      the x-coordinate of the left-bottom point
     * @param y      the x-coordinate of the left-bottom point
     * @param width  the width
     * @param height the height
     * @throws IOException
     */
    @SuppressWarnings("boxing")
    public static void writeRectangle(BufferedWriter bw, double x, double y, double width, double height) throws IOException {
        bw.write(String.format("%d\n", 4));
        bw.write(String.format("%f %f\n", x, y));
        bw.write(String.format("%f %f\n", x + width, y));
        bw.write(String.format("%f %f\n", x + width, y + height));
        bw.write(String.format("%f %f\n", x, y + height));
    }
    
    public static void planarRobotEasyEnvironment(double[] xBounds, double[] yBounds, String filename) throws IOException {
        Charset charset = Charset.forName("US-ASCII");
        Path file = FileSystems.getDefault().getPath(".", filename);
        int numberOfObstacles = 2;
        try (BufferedWriter bw = Files.newBufferedWriter(file, charset)) {
            bw.write(String.format("%d\n", numberOfObstacles + 4));
            double w = xBounds[1] - xBounds[0];
            double h = yBounds[1] - yBounds[0];
            writeRectangle(bw, xBounds[0], yBounds[0], w, 0.1);
            writeRectangle(bw, xBounds[0], yBounds[1], w, 0.1);
            writeRectangle(bw, xBounds[0], yBounds[0], 0.1, h);
            writeRectangle(bw, xBounds[1], yBounds[0], 0.1, h);
            writeRectangle(bw, -3, -3, 6, 0.1);
            writeRectangle(bw, -3, 3, 6, 0.1);
        }
    }
    
    public static void planarRobotHardEnvironment(double[] xBounds, double[] yBounds, String filename) throws IOException {
        Charset charset = Charset.forName("US-ASCII");
        Path file = FileSystems.getDefault().getPath(".", filename);
        int numberOfObstacles = 6;
        try (BufferedWriter bw = Files.newBufferedWriter(file, charset)) {
            bw.write(String.format("%d\n", numberOfObstacles + 4));
            double w = xBounds[1] - xBounds[0];
            double h = yBounds[1] - yBounds[0];
            writeRectangle(bw, xBounds[0], yBounds[0], w, 0.1);
            writeRectangle(bw, xBounds[0], yBounds[1], w, 0.1);
            writeRectangle(bw, xBounds[0], yBounds[0], 0.1, h);
            writeRectangle(bw, xBounds[1], yBounds[0], 0.1, h);
            writeRectangle(bw, -5, -3, 8, 0.1);
            writeRectangle(bw, -3, -1, 8, 0.1);
            writeRectangle(bw, -3, -1, 0.1, 4);
            writeRectangle(bw, -1, 1, 0.1, 4);
            writeRectangle(bw, 1, -1, 0.1, 4);
            writeRectangle(bw, 3, 1, 0.1, 4);
        }
    }
    
    public static void robotArmEasyEnvironment(double[] xBounds, double[] yBounds, String filename) throws IOException {
        Charset charset = Charset.forName("US-ASCII");
        Path file = FileSystems.getDefault().getPath(".", filename);
        int numberOfObstacles = 4;
        try (BufferedWriter bw = Files.newBufferedWriter(file, charset)) {
            bw.write(String.format("%d\n", numberOfObstacles + 4));
            double w = xBounds[1] - xBounds[0];
            double h = yBounds[1] - yBounds[0];
            writeRectangle(bw, xBounds[0], yBounds[0], w, 0.1);
            writeRectangle(bw, xBounds[0], yBounds[1], w, 0.1);
            writeRectangle(bw, xBounds[0], yBounds[0], 0.1, h);
            writeRectangle(bw, xBounds[1], yBounds[0], 0.1, h);
            writeRectangle(bw, 2, 2, 0.5, 0.5);
            writeRectangle(bw, 2, -2, 0.5, 0.5);
            writeRectangle(bw, -2, 2, 0.5, 0.5);
            writeRectangle(bw, -2, -2, 0.5, 0.5);
        }
    }
    
    public static void robotArmHardEnvironment(double[] xBounds, double[] yBounds, String filename) throws IOException {
        Charset charset = Charset.forName("US-ASCII");
        Path file = FileSystems.getDefault().getPath(".", filename);
        int numberOfObstacles = 10;
        try (BufferedWriter bw = Files.newBufferedWriter(file, charset)) {
            bw.write(String.format("%d\n", numberOfObstacles + 4));
            double w = xBounds[1] - xBounds[0];
            double h = yBounds[1] - yBounds[0];
            writeRectangle(bw, xBounds[0], yBounds[0], w, 0.1);
            writeRectangle(bw, xBounds[0], yBounds[1], w, 0.1);
            writeRectangle(bw, xBounds[0], yBounds[0], 0.1, h);
            writeRectangle(bw, xBounds[1], yBounds[0], 0.1, h);
            writeRectangle(bw, 2, 2, 0.5, 0.5);
            writeRectangle(bw, 2, -2, 0.5, 0.5);
            writeRectangle(bw, -2, 2, 0.5, 0.5);
            writeRectangle(bw, -2, -2, 0.5, 0.5);
            writeRectangle(bw, 3, -1, 0.5, 0.5);
            writeRectangle(bw, 3, -3, 0.5, 0.5);
            writeRectangle(bw, 1, 1, 0.5, 0.5);
            writeRectangle(bw, 1, -1.8, 0.5, 0.5);
            writeRectangle(bw, -1, -1, 0.5, 0.5);
            writeRectangle(bw, 1.8, 0.5, 0.5, 0.5);
        }
    }
    
    public static void main(String[] args) {
        try {
            //MazeGenerator.generate(10, new double[]{-5, 5}, new double[]{-5, 5}, "ob");
            //MazeGenerator.planarRobotHardEnvironment(new double[]{-5, 5}, new double[]{-5, 5}, "planar_robot_environment_hard");
            //MazeGenerator.planarRobotEasyEnvironment(new double[]{-5, 5}, new double[]{-5, 5}, "planar_robot_environment");
            //MazeGenerator.robotArmEasyEnvironment(new double[]{-5, 5}, new double[]{-5, 5}, "robot_arm_environment");
            MazeGenerator.robotArmHardEnvironment(new double[]{-5, 5}, new double[]{-5, 5}, "robot_arm_environment_hard");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
