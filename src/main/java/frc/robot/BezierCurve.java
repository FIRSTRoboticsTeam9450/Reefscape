package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BezierCurve {

    private double[] bezier;

    private double x1;
    private double y1;
    private double x2;
    private double y2;
    private double deadband = 0.05; // joystick deadband
    private double minOutput = .1;
    private String identifier;
    private double joystick = -1;

    private String string_x1;
    private String string_y1;
    private String string_x2;
    private String string_y2;
    private String string_deadband;
    private String string_minOutput;
    private String string_waveformX;   
    private String string_waveformY;

    private String string_update;
    private String base = "Reefscape/DriveCurve/";
    private String update = "_update";

    public BezierCurve(String identifier, double x1, double y1, double x2, double y2, double deadband, double minOutput){
        this.identifier = identifier;
        this.x1 = x1;
        this.y1 = y1;
        this.x2 = x2;
        this.y2 = y2;
        this.deadband = deadband;
        this.minOutput = minOutput;

        string_x1 = base+identifier+"_x1";
        string_y1 = base+identifier+"_y1";
        string_x2 = base+identifier+"_x2";
        string_y2 = base+identifier+"_y2";
        string_deadband = base+identifier+"_deadband";
        string_minOutput = base+identifier+"_minOutput";
        string_waveformX = base+identifier+"_waveform_X";
        string_waveformY = base+identifier+"_waveform_Y";
        string_update = base+identifier+"_update";

        generateCurve(x1, y1, x2, y2, minOutput, deadband);
    }

    public double getOutput(double x) {
        x = MathUtil.clamp(x, -1, 1);
        double sign = Math.signum(x);
        x = Math.abs(x);
        // if (x < deadband){
        //     return 0;
        // }
        // x = x-deadband;
        // x /= (1 - deadband);

        // x = x+minOutput;
        // x /= (1 +minOutput);

        return bezier[(int) Math.round(x * 127)] * sign;
    }

    private double getPoint(double[][] curve, double x) {
        double sign = Math.signum(x);
        x = Math.abs(x);

        if (x / 127.0 < deadband){
            x = 0;
        } else {
            x = 127 * (x / 127 - deadband);
            x /= (1.0 - deadband);
        }

        for (int i = 0; i < curve[0].length; i++) {
            if (x <= curve[0][i]) {
                if (x == 0) {
                    return 0;
                }
                double x1 = curve[0][i-1];
                double y1 = curve[1][i-1];
                double x2 = curve[0][i];
                double y2 = curve[1][i];
                double out = (y1 + ((x - x1) * (y2 - y1)) / (x2 - x1));

                out = out + minOutput;
                out /= (1.0 + minOutput);
                return out * sign;
            }
        }
        
        return 0;
    }

    public void generateCurve(double px1, double py1, double px2, double py2, double pminOutput, double pdeadband) {
        x1 = px1;
        y1 = py1;
        x2 = px2;
        y2 = py2;
        minOutput = pminOutput;
        deadband = pdeadband;

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

        bezier = out;
    }
    
    private void logCurve() {
        // Logger.recordOutput(string_waveformX, joystick);
        // Logger.recordOutput(string_waveformY, getOutput(joystick));
        SmartDashboard.putNumber(string_waveformY, getOutput(joystick));
        joystick += .01;
        if (joystick > 1) {
            joystick = -1;
        }
    }

    public void dashboardInitialSettings(){
        double testValue = SmartDashboard.getNumber(string_x1, -100.0);

        SmartDashboard.putBoolean(base+identifier+update, false);

        if (testValue == -100.0) {
            SmartDashboard.putNumber(string_x1, x1);
            SmartDashboard.putNumber(string_y1, y1);
            SmartDashboard.putNumber(string_x2, x2);
            SmartDashboard.putNumber(string_y2, y2);
            SmartDashboard.putNumber(string_deadband, deadband);
            SmartDashboard.putNumber(string_minOutput, minOutput);
        }
        else{
            x1 = SmartDashboard.getNumber(string_x1, 0);
            y1 = SmartDashboard.getNumber(string_y1, 0);
            x2 = SmartDashboard.getNumber(string_x2, 0);
            y2 = SmartDashboard.getNumber(string_y2, 0);
            deadband = SmartDashboard.getNumber(string_deadband, 0);
            minOutput = SmartDashboard.getNumber(string_minOutput, 0);
        }
    }

    int count=0;
    public void checkAndupdateCurve(){
        if(((count++) % 25) == 0){
            
            boolean bUpdate = SmartDashboard.getBoolean(string_update,false);
            if (bUpdate) {

                SmartDashboard.putBoolean(string_update, false);

                double tmpx1 = SmartDashboard.getNumber(string_x1, 0);
                double tmpy1 = SmartDashboard.getNumber(string_y1, 0);
                double tmpx2 = SmartDashboard.getNumber(string_x2, 0);
                double tmpy2 = SmartDashboard.getNumber(string_y2, 0);
                double tmpdeadband = SmartDashboard.getNumber(string_deadband, 0);
                double tmpminOutput = SmartDashboard.getNumber(string_minOutput, 0);

                
                //System.out.printf("x1: %f \ntmpx1: %f\n", x1, tmpx1);
                System.out.println("Updated " + identifier + " at " + Utils.getCurrentTimeSeconds());

                //tmpx1 != x1 || tmpx2 != x2 || tmpy1 != y1 || tmpy2 != y2 || tmpdeadband != deadband || tmpminOutput != minOutput
                //System.out.println("checkAndUpdateCurve");
                x1 = tmpx1;
                x2 = tmpx2;
                y1 = tmpy1;
                y2 = tmpy2;
                deadband = tmpdeadband;
                minOutput = tmpminOutput;

                generateCurve(x1, y1, x2, y2, minOutput, deadband);
                //System.out.println("Hello there!");
            }
        }
        logCurve();
    }
}
