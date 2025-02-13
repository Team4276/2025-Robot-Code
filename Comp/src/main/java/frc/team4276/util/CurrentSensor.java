package frc.team4276.util;

import java.util.List;
import java.util.function.Supplier;

import org.apache.commons.math3.stat.StatUtils;
import org.littletonrobotics.junction.Logger;

import frc.team4276.util.dashboard.LoggedTunableNumber;

public class CurrentSensor {
    Supplier<Double> mCurrentSuplier;
    List<Double> currentSamples;
    Double currentAverage;

    boolean spikeDetected;
    public CurrentSensor(Supplier<Double> currentSuplier){
        mCurrentSuplier = currentSuplier;
    }
    public void update(){
        spikeDetected = false;
        Double current = mCurrentSuplier.get();
        Logger.recordOutput("ObjectSensor/current", current);


        if(current < new LoggedTunableNumber("ObjectSensor/currentOutlierValue", 45).getAsDouble()){
            currentSamples.add(mCurrentSuplier.get());
        }
        if(currentSamples.size() > 500){
            currentSamples.remove(0);
            currentAverage = StatUtils.mean(currentSamples.stream().mapToDouble(Double::doubleValue).toArray());
            spikeDetected = current > currentAverage + new LoggedTunableNumber("ObjectSensor/significantCurrentSpike", 10).getAsDouble();
        }else{
            spikeDetected = false;
        }
    }
    public boolean getDetection(){
        return spikeDetected;
    }
}
