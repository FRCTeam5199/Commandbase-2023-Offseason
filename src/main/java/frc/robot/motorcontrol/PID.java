package frc.robot.motorcontrol;

public class PID {
    
    private static final PID EMPTY_PID = new PID(0, 0, 0, 0);
    public final double P, I, D, F;

    public PID(double p, double i, double d){
        this(p, i, d, 0);
    }

    public PID(double p, double i, double d, double f){
        P = p;
        I = i;
        D = d;
        F = f;
    }

    public String printPID(){
        return("PIDF: P is " + P + ",I is " + I + ", D is " + D + ", F is " + F );
    }
    
}
