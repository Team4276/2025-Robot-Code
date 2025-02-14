package frc.team4276.util.drivers;

import java.util.List;
import org.apache.commons.math3.stat.StatUtils;
import org.littletonrobotics.junction.Logger;

import frc.team4276.util.dashboard.LoggedTunableNumber;

public class CurrentSensor {
  private final String key;

  private final LoggedTunableNumber spikeThreshold;
  private final LoggedTunableNumber outlierThreshold;
  private List<Double> currentSamples;
  private double currentAverage;

  boolean spikeDetected;

  public CurrentSensor(String key) {
    this.key = key;

    spikeThreshold = new LoggedTunableNumber(key + "/CurrentSensor/significantCurrentSpike", 10);
    outlierThreshold = new LoggedTunableNumber(key + "/CurrentSensor/currentOutlierValue", 45);
  }

  public void update(double current) {
    spikeDetected = false;
    Logger.recordOutput(key + "/CurrentSensor/current", current);

    if (current < outlierThreshold.getAsDouble()) {
      currentSamples.add(current);
    }

    if (currentSamples.size() > 500) {
      currentSamples.remove(0);
      currentAverage = StatUtils.mean(currentSamples.stream().mapToDouble(Double::doubleValue).toArray());
      spikeDetected = current > currentAverage
          + spikeThreshold.getAsDouble();
    } else {
      spikeDetected = false;
    }
  }

  public boolean getDetection() {
    return spikeDetected;
  }
}
