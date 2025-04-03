package frc.robot;
import java.util.HashMap;
import java.util.Scanner;

public class AlignHelper {

    private static HashMap<Integer, double[]> map = new HashMap<>();
	
	public static void main(String[] args) {
		generateMap();
		Scanner console = new Scanner(System.in);
		boolean running = true;
		while (running) {
			System.out.println("1 - get alignment position\n2 - calculate offsets\n3 - quit");
			
			System.out.print("Select an option -> ");
			int option = console.nextInt();
			
			if (option == 1) {
				System.out.print("\nEnter target tag: ");
				int tag = console.nextInt();
				if (map.containsKey(tag)) {
					System.out.printf("Tag %s location\n", tag);
					printLocation(map.get(tag));
					
					System.out.println("\nLeft");
					printLocation(getAlignPos(map.get(tag), AlignPos.LEFT));
					
					System.out.println("\nRight");
					printLocation(getAlignPos(map.get(tag), AlignPos.RIGHT));
					
					System.out.println("\nCenter");
					printLocation(getAlignPos(map.get(tag), AlignPos.CENTER));
					System.out.println("\n");
				}
			} else if (option == 2) {
				System.out.print("\nEnter target tag: ");
				int tag = console.nextInt();
				if (map.containsKey(tag)) {
					double[] robotPos = new double[2];
					System.out.print("Enter robot X -> ");
					robotPos[0] = console.nextDouble();
					System.out.print("Enter robot Y -> ");
					robotPos[1] = console.nextDouble();
					
					double[] tagPos = map.get(tag);
					
					double x = tagPos[0] - robotPos[0];
					double y = tagPos[1] - robotPos[1];
					
					double forward = x * Math.cos(tagPos[2]) + y * Math.sin(tagPos[2]);
					double left = -x * Math.sin(tagPos[2]) + y * Math.cos(tagPos[2]);
					
					System.out.printf("\nForward offset: %s\nLeft offset: %s\n\n\n", -forward, left);
					
				}
			} else {
				running = false;
			}
		}
		
		console.close();
	}
	
	public static void printLocation(double[] coordinates) {
		double angle = coordinates[2] / Math.PI * 180;
		System.out.printf("X: %f\nY: %f\nAngle: %s\n", coordinates[0], coordinates[1], angle);
	}
	
	public static double[] getAlignPos(double[] targetPos, AlignPos position) {
        //double tagForwardOffset = 0.46;
        //double tagLeftOffset = 0.13;

		// Forward offset: 0.45561842367428596
		// Left offset: 0.15651825866830574
		double tagForwardOffset = 0.44;
		double tagLeftOffset = 0.173;
        if (position == AlignPos.RIGHT) {
            tagLeftOffset = -0.173;
        }
        //System.out.println(score.getPos());
        if (position == AlignPos.CENTER) {
            tagLeftOffset = 0; // Set left offset for center
            tagForwardOffset = 0.75; ; // Set forward offset for center
        }

        // Calculate rotation relative to the target position
        double rotation = targetPos[2] - Math.PI;

        // Normalize the rotation to ensure it wraps around for the shortest distance
        if (rotation < -Math.PI) {
            rotation += 2 * Math.PI;
        } else if (rotation > Math.PI) {
            rotation -= 2 * Math.PI;
        }

        // Calculate the new x and y coordinates based on the offsets and rotation
        double x = targetPos[0] - tagForwardOffset * Math.cos(rotation) - tagLeftOffset * Math.sin(rotation);
        double y = targetPos[1] - tagForwardOffset * Math.sin(rotation) + tagLeftOffset * Math.cos(rotation);

        // Create an array with the calculated x, y, and rotation values and return it
        double[] out = {x, y, rotation};
        return out;
    }
	
	public static void generateMap() {
		double[] tag6 = {13.474446, 3.306318, 5 * Math.PI / 3.0};
        double[] tag7 = {13.890498, 4.0259, 0};
        double[] tag8 = {13.474446, 4.745482, Math.PI / 3.0};
        double[] tag9 = {12.643358, 4.745482, 2 * Math.PI / 3.0};
        double[] tag10 = {12.227306, 4.0259, Math.PI};
        double[] tag11 = {12.643358, 3.306318, 4 * Math.PI / 3.0};
        double[] tag17 = {4.0739, 3.3063, 4 * Math.PI / 3.0};
        double[] tag18 = {3.6576, 4.0259, Math.PI};
        double[] tag19 = {4.0739, 4.7455, 2 * Math.PI / 3.0};
        double[] tag20 = {4.9047, 4.7455, Math.PI / 3.0};
        double[] tag21 = {5.3210, 4.0259, 0};
        double[] tag22 = {4.9047, 3.3063, 5 * Math.PI / 3.0};
        map.put(6, tag6);
        map.put(7, tag7);
        map.put(8, tag8);
        map.put(9, tag9);
        map.put(10, tag10);
        map.put(11, tag11);
        map.put(17, tag17);
        map.put(18, tag18);
        map.put(19, tag19);
        map.put(20, tag20);
        map.put(21, tag21);
        map.put(22, tag22);
	}

	public enum AlignPos {
		LEFT,
		RIGHT,
		CENTER
	}
	
}
