package frc.team4276.util;
import java.util.List;
import java.util.function.Supplier;

import org.apache.commons.math3.stat.StatUtils;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.team4276.util.dashboard.LoggedTunableNumber;


public class VelocitySensor {
    Supplier<Double> mAccSupplier;
    Supplier<Double> mVelocitySupplier;
    List<Double> velocitySamples;
    Double averageVelocity;
    boolean dipDetected;
    Double timeOld;
    public VelocitySensor(Supplier<Double> velocitySupplier){
        mVelocitySupplier = velocitySupplier;
        timeOld = Timer.getTimestamp();
    }
    //TODO: make these constant easier to config 
    public void update(){
        double timeNow = Timer.getTimestamp();

        dipDetected = false;
        double velocity = mVelocitySupplier.get();
        double acceleration = 0;
        if(velocitySamples.isEmpty()){
            acceleration = velocity;
        }else{
            acceleration = (velocity - velocitySamples.get(velocitySamples.size() - 1)) / (timeNow - timeOld);
        }
        //no longer accelarting but not at rest 
        Logger.recordOutput("ObjectSensor/Acceleration", acceleration);
        Logger.recordOutput("ObjectSensor/Velocity", velocity);

        if (withinRange(
            acceleration, 
        new LoggedTunableNumber("ObjectSensor/lowerboundAccRange", -0.2).getAsDouble(), 
        new LoggedTunableNumber("ObjectSensor/upperBoundAccRange", 0.2).getAsDouble()
        ) && velocity >= 30) {
            velocitySamples.add(velocity);
        }
        if(velocitySamples.size() > 500){
            averageVelocity = StatUtils.mean(velocitySamples.stream().mapToDouble(Double::doubleValue).toArray());
            velocitySamples.remove(0);
            dipDetected = (velocity < averageVelocity - new LoggedTunableNumber("ObjectSensor/significantVelDipRange", 10).getAsDouble()) 
            && acceleration < new LoggedTunableNumber("ObjectSensor/deaccelerationRange", -1.0).getAsDouble();
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
