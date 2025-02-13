package frc.team4276.util;
import java.util.List;
import java.util.function.Supplier;

import org.apache.commons.math3.stat.StatUtils;

public class VelocitySensor {
    Supplier<Double> mAccSupplier;
    Supplier<Double> mVelocitySupplier;
    List<Double> velocitySamples;
    Double averageVelocity;
    boolean dipDetected;

    public VelocitySensor(Supplier<Double> accSupplier, Supplier<Double> velocitySupplier){
        mAccSupplier = accSupplier;
        mVelocitySupplier = velocitySupplier;
    }
    //TODO: make these constant easier to config 
    public void update(){
        dipDetected = false;
        double acceleration = mAccSupplier.get();
        double velocity = mVelocitySupplier.get();
        //no longer accelarting but not at rest 
        if (withinRange(acceleration, -0.2, 0.2 ) && velocity >= 30) {
            velocitySamples.add(velocity);
        }
        if(velocitySamples.size() > 500){
            averageVelocity = StatUtils.mean(velocitySamples.stream().mapToDouble(Double::doubleValue).toArray());
            velocitySamples.remove(0);
            dipDetected = (velocity < averageVelocity - 10) && acceleration < -1;
        }

    }
    public boolean getDip(){
        return dipDetected;
    }
    //maybe move to its own thing just a lil helper function cuz i hate doing these 
    private static boolean withinRange(double value, double min, double max) {
        return value >= min && value <= max;
    }
    
}
