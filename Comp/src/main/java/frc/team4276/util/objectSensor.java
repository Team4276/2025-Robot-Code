package frc.team4276.util;

import java.util.function.Supplier;


public class objectSensor {
    CurrentSensor cSensor;
    VelocitySensor vSensor;

    boolean objectDetected;

    objectSensor(Supplier<Double> currentSupplier, Supplier<Double> velocitySupplier, Supplier<Double> accelerationSupplier){
        cSensor = new CurrentSensor(currentSupplier);
        vSensor = new VelocitySensor(accelerationSupplier, accelerationSupplier);
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
