package frc.robot;

public class CurveTest {
    public static void main(String[] args) {
        double[] curve = generateCurve();
        for (int i = 0; i < curve.length; i++) {
            System.out.println(curve[i]);
        }
        System.out.println(getOutput(curve, 0.5));
    }

    public static double getOutput(double[] curve, double x) {
        return curve[(int) Math.round(x * 127)];
    }

    public static double getPoint(double[][] curve, double x) {
        double sign = Math.signum(x);
        x = Math.abs(x);

        for (int i = 0; i < curve[0].length; i++) {
            if (x <= curve[0][i]) {
                if (x == 0) {
                    return 0;
                }
                double x1 = curve[0][i-1];
                double y1 = curve[1][i-1];
                double x2 = curve[0][i];
                double y2 = curve[1][i];
                return sign * (y1 + ((x - x1) * (y2 - y1)) / (x2 - x1));
            }
        }
        
        return 0;
    }

    public static double[] generateCurve() {
        double x1 = 26.3;
        double y1 = 0.7;
        double x2 = 73.4;
        double y2 = 0.87;

        int steps = 1024;
        double[][] curve = new double[2][steps + 1];
        for (int i = 0; i <= steps; i++) {
            double t = (double)i / steps;
            curve[0][i] = x1 * 3 * Math.pow(1 - t, 2) * t + x2 * 3 * (1 - t) * Math.pow(t, 2) + 127 * Math.pow(t, 3);
            curve[1][i] = y1 * 3 * Math.pow(1 - t, 2) * t + y2 * 3 * (1 - t) * Math.pow(t, 2) + Math.pow(t, 3);
            //System.out.println(curve[0][i] + "," + curve[1][i]);
        }

        double[] out = new double[128];
        for (int i = 0; i < 128; i++) {
            out[i] = getPoint(curve, i);
        }

        return out;
    }
}
