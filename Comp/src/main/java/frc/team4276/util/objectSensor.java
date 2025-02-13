package frc.team4276.util;

import java.util.function.Supplier;


public class objectSensor {
    CurrentSensor cSensor;
    VelocitySensor vSensor;

    boolean objectDetected;

    public objectSensor(Supplier<Double> currentSupplier, Supplier<Double> velocitySupplier){
        cSensor = new CurrentSensor(currentSupplier);
        vSensor = new VelocitySensor(velocitySupplier);
    }
    public void update(){
        cSensor.update();
        vSensor.update();
        objectDetected = cSensor.getDetection() && vSensor.getDip();
    }
    public boolean getDetection(){
        return objectDetected;
    }
    
}
