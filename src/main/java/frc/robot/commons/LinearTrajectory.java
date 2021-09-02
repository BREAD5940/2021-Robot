package frc.robot.commons;

import java.util.List;
import java.util.stream.Collectors;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;

// Linear trajectory class
public class LinearTrajectory {

    // Variables
    final List<Trajectory> lines;
    final List<Double> totalTimesSeconds;

    // Constructor
    public LinearTrajectory(List<Trajectory> lines) {
        this.lines = lines;
        totalTimesSeconds = lines.stream().map(line -> line.getTotalTimeSeconds()).collect(Collectors.toList());
    }

    // Method to get the time of this trajectory in seconds
    public double getTotalTimeSeconds() {
        double time = 0.0;
        for (double timeSeconds : totalTimesSeconds) {
            time += timeSeconds;
        }
        return time;
    }

    // Method to sample the trajectory
    public State sample(double time) {
        if (time < 0.0) {
            return lines.get(0).getStates().get(0);
        } else if (time >= getTotalTimeSeconds()) {
            return lines.get(lines.size() - 1).getStates().get(lines.get(lines.size() - 1).getStates().size() - 1);
        } else {
            int currLine = 0;
            double timeElapsed = lines.get(0).getTotalTimeSeconds();
            while (time > timeElapsed) {
                currLine++;
                timeElapsed += lines.get(currLine).getTotalTimeSeconds();
            }
            return lines.get(currLine).sample(time - timeElapsed + lines.get(currLine).getTotalTimeSeconds());
        }
    }

}