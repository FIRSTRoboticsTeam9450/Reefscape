package frc.robot;

import java.util.HashMap;
import java.util.Scanner;

/**
 * AlignHelper is a utility class for calculating alignment positions
 * and offsets between a robot and AprilTags on the field.
 */
public class AlignHelper {

    // Stores tag IDs and their associated position arrays: [x, y, rotation]
    private static HashMap<Integer, double[]> map = new HashMap<>();

    public static void main(String[] args) {
        generateMap(); // Populate tag map
        Scanner console = new Scanner(System.in);
        boolean running = true;

        // Command loop
        while (running) {
            System.out.println("1 - Get alignment position");
            System.out.println("2 - Calculate offsets");
            System.out.println("3 - Quit");

            System.out.print("Select an option -> ");
            int option = console.nextInt();

            switch (option) {
                case 1:
                    // Display alignment positions
                    System.out.print("\nEnter target tag: ");
                    int tag = console.nextInt();
                    if (map.containsKey(tag)) {
                        System.out.printf("Tag %d location\n", tag);
                        printLocation(map.get(tag));

                        System.out.println("\nLeft:");
                        printLocation(getAlignPos(map.get(tag), AlignPos.LEFT));

                        System.out.println("\nRight:");
                        printLocation(getAlignPos(map.get(tag), AlignPos.RIGHT));

                        System.out.println("\nCenter:");
                        printLocation(getAlignPos(map.get(tag), AlignPos.CENTER));
                        System.out.println();
                    }
                    break;

                case 2:
                    // Calculate robot-to-tag positional offsets
                    System.out.print("\nEnter target tag: ");
                    tag = console.nextInt();
                    if (map.containsKey(tag)) {
                        double[] robotPos = new double[2];

                        System.out.print("Enter robot X -> ");
                        robotPos[0] = console.nextDouble();

                        System.out.print("Enter robot Y -> ");
                        robotPos[1] = console.nextDouble();

                        double[] tagPos = map.get(tag);

                        // Translate robot position relative to tag's orientation
                        double dx = tagPos[0] - robotPos[0];
                        double dy = tagPos[1] - robotPos[1];

                        double forward = dx * Math.cos(tagPos[2]) + dy * Math.sin(tagPos[2]);
                        double left = -dx * Math.sin(tagPos[2]) + dy * Math.cos(tagPos[2]);

                        System.out.printf("\nForward offset: %.3f\nLeft offset: %.3f\n\n", -forward, left);
                    }
                    break;

                default:
                    running = false;
                    break;
            }
        }

        console.close();
    }

    /**
     * Prints a formatted location (X, Y, angle in degrees)
     */
    public static void printLocation(double[] coordinates) {
        double angleDegrees = coordinates[2] * 180.0 / Math.PI;
        System.out.printf("X: %.3f\nY: %.3f\nAngle: %.1f°\n", coordinates[0], coordinates[1], angleDegrees);
    }

    /**
     * Computes a position offset based on alignment side
     */
    public static double[] getAlignPos(double[] targetPos, AlignPos position) {
        // Retrieve default offsets
        double tagForwardOffset = Constants.robotConstants.AlignOffsets.scoreCoralBack;
        double tagLeftOffset = Constants.robotConstants.AlignOffsets.leftReef;

        if (position == AlignPos.RIGHT) {
            tagLeftOffset = Constants.robotConstants.AlignOffsets.rightReef;
        } else if (position == AlignPos.CENTER) {
            tagForwardOffset = Constants.robotConstants.AlignOffsets.algaeBack;
            tagLeftOffset = Constants.robotConstants.AlignOffsets.algaeLeft;
        }

        // Calculate robot rotation relative to the tag
        double rotation = targetPos[2] - Math.PI;

        // Normalize angle to [-π, π]
        if (rotation < -Math.PI) rotation += 2 * Math.PI;
        else if (rotation > Math.PI) rotation -= 2 * Math.PI;

        // Offset the target position by applying rotation
        double x = targetPos[0] - tagForwardOffset * Math.cos(rotation) - tagLeftOffset * Math.sin(rotation);
        double y = targetPos[1] - tagForwardOffset * Math.sin(rotation) + tagLeftOffset * Math.cos(rotation);

        return new double[]{x, y, rotation};
    }

    /**
     * Populates tag positions into the map
     */
    public static void generateMap() {
        map.put(6, new double[]{13.474446, 3.306318, 5 * Math.PI / 3});
        map.put(7, new double[]{13.890498, 4.0259, 0});
        map.put(8, new double[]{13.474446, 4.745482, Math.PI / 3});
        map.put(9, new double[]{12.643358, 4.745482, 2 * Math.PI / 3});
        map.put(10, new double[]{12.227306, 4.0259, Math.PI});
        map.put(11, new double[]{12.643358, 3.306318, 4 * Math.PI / 3});
        map.put(17, new double[]{4.0739, 3.3063, 4 * Math.PI / 3});
        map.put(18, new double[]{3.6576, 4.0259, Math.PI});
        map.put(19, new double[]{4.0739, 4.7455, 2 * Math.PI / 3});
        map.put(20, new double[]{4.9047, 4.7455, Math.PI / 3});
        map.put(21, new double[]{5.3210, 4.0259, 0});
        map.put(22, new double[]{4.9047, 3.3063, 5 * Math.PI / 3});
    }

    /**
     * Enum representing alignment sides
     */
    public enum AlignPos {
        LEFT,
        RIGHT,
        CENTER
    }
}