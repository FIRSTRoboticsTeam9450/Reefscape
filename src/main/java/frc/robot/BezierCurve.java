package frc.robot;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * BezierCurve gets and manages joystick response curves using cubic Bezier interpolation.
 */
public class BezierCurve {

    // Bezier output lookup table (0–127 sampled response curve)
    private double[] bezier;

    // Bezier control points
    private double x1, y1, x2, y2;

    // Joystick deadband and output shaping
    private double deadband = 0.05;
    private double minOutput = 0.1;

    // Dashboard identifiers and state
    private final String identifier;
    private double joystick = -1;
    private final String base = "Reefscape/DriveCurve/";

    // SmartDashboard keys
    private final String string_x1, string_y1, string_x2, string_y2;
    private final String string_deadband, string_minOutput;
    private final String string_waveformY;
    private final String string_update;

    public BezierCurve(String identifier, double x1, double y1, double x2, double y2, double deadband, double minOutput) {
        this.identifier = identifier;
        this.x1 = x1;
        this.y1 = y1;
        this.x2 = x2;
        this.y2 = y2;
        this.deadband = deadband;
        this.minOutput = minOutput;

        // Construct SmartDashboard keys
        string_x1 = base + identifier + "_x1";
        string_y1 = base + identifier + "_y1";
        string_x2 = base + identifier + "_x2";
        string_y2 = base + identifier + "_y2";
        string_deadband = base + identifier + "_deadband";
        string_minOutput = base + identifier + "_minOutput";
        string_waveformY = base + identifier + "_waveform_Y";
        string_update = base + identifier + "_update";

        // Precompute Bezier curve output table
        generateCurve(x1, y1, x2, y2, minOutput, deadband);
    }

    /**
     * Returns output value for input `x` (clamped between -1 and 1)
     */
    public double getOutput(double x) {
        x = MathUtil.clamp(x, -1, 1);
        double sign = Math.signum(x);
        x = Math.abs(x);
        return bezier[(int) Math.round(x * 127)] * sign;
    }

    /**
     * Evaluates interpolated output for a given input sample on the generated curve
     */
    private double getPoint(double[][] curve, double x) {
        double sign = Math.signum(x);
        x = Math.abs(x);

        if (x / 127.0 < deadband) {
            x = 0;
        } else {
            x = 127 * (x / 127 - deadband);
            x /= (1.0 - deadband);
        }

        for (int i = 1; i < curve[0].length; i++) {
            if (x <= curve[0][i]) {
                double x1 = curve[0][i - 1];
                double y1 = curve[1][i - 1];
                double x2 = curve[0][i];
                double y2 = curve[1][i];
                double out = y1 + ((x - x1) * (y2 - y1)) / (x2 - x1);

                out = (out + minOutput) / (1.0 + minOutput);
                return out * sign;
            }
        }
        return 0;
    }

    /**
     * Generates the Bezier curve lookup table using control points and settings
     */
    public void generateCurve(double px1, double py1, double px2, double py2, double pminOutput, double pdeadband) {
        this.x1 = px1;
        this.y1 = py1;
        this.x2 = px2;
        this.y2 = py2;
        this.minOutput = pminOutput;
        this.deadband = pdeadband;

        int steps = 1024;
        double[][] curve = new double[2][steps + 1];
        for (int i = 0; i <= steps; i++) {
            double t = (double) i / steps;
            curve[0][i] = x1 * 3 * Math.pow(1 - t, 2) * t +
                          x2 * 3 * (1 - t) * Math.pow(t, 2) +
                          127 * Math.pow(t, 3);
            curve[1][i] = y1 * 3 * Math.pow(1 - t, 2) * t +
                          y2 * 3 * (1 - t) * Math.pow(t, 2) +
                          Math.pow(t, 3);
        }

        double[] out = new double[128];
        for (int i = 0; i < 128; i++) {
            out[i] = getPoint(curve, i);
        }
        bezier = out;
    }

    /**
     * Logs curve output to SmartDashboard for visualization
     */
    private void logCurve() {
        SmartDashboard.putNumber(string_waveformY, getOutput(joystick));
        joystick += 0.01;
        if (joystick > 1) joystick = -1;
    }

    /**
     * Initializes curve parameters from SmartDashboard or stores default values
     */
    public void dashboardInitialSettings() {
        double testValue = SmartDashboard.getNumber(string_x1, -100.0);
        SmartDashboard.putBoolean(string_update, false);

        if (testValue == -100.0) {
            SmartDashboard.putNumber(string_x1, x1);
            SmartDashboard.putNumber(string_y1, y1);
            SmartDashboard.putNumber(string_x2, x2);
            SmartDashboard.putNumber(string_y2, y2);
            SmartDashboard.putNumber(string_deadband, deadband);
            SmartDashboard.putNumber(string_minOutput, minOutput);
        } else {
            x1 = SmartDashboard.getNumber(string_x1, 0);
            y1 = SmartDashboard.getNumber(string_y1, 0);
            x2 = SmartDashboard.getNumber(string_x2, 0);
            y2 = SmartDashboard.getNumber(string_y2, 0);
            deadband = SmartDashboard.getNumber(string_deadband, 0);
            minOutput = SmartDashboard.getNumber(string_minOutput, 0);
        }
    }

    private int count = 0;

    /**
     * Checks for update signal and refreshes curve settings accordingly
     */
    public void checkAndupdateCurve() {
        if ((count++ % 25) == 0) {
            boolean bUpdate = SmartDashboard.getBoolean(string_update, false);

            if (bUpdate) {
                SmartDashboard.putBoolean(string_update, false);

                double tmpx1 = SmartDashboard.getNumber(string_x1, 0);
                double tmpy1 = SmartDashboard.getNumber(string_y1, 0);
                double tmpx2 = SmartDashboard.getNumber(string_x2, 0);
                double tmpy2 = SmartDashboard.getNumber(string_y2, 0);
                double tmpdeadband = SmartDashboard.getNumber(string_deadband, 0);
                double tmpminOutput = SmartDashboard.getNumber(string_minOutput, 0);

                System.out.println("Updated " + identifier + " at " + Utils.getCurrentTimeSeconds());

                x1 = tmpx1;
                y1 = tmpy1;
                x2 = tmpx2;
                y2 = tmpy2;
                deadband = tmpdeadband;
                minOutput = tmpminOutput;

                generateCurve(x1, y1, x2, y2, minOutput, deadband);
            }
        }
        logCurve();
    }
}