package frc.robot.interpolation;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;
import static java.util.Map.entry;

// Interpolating table
public class InterpolatingTable {

    /* Private constructor because this is a utility class */
    private InterpolatingTable() {}

    // Interpolating tree map
    private static final TreeMap<Double, ShotParameter> map = new TreeMap<>(
        Map.ofEntries(
            entry(1.97, new ShotParameter(11, 2750, 0.0)),
            entry(2.77, new ShotParameter(18, 3000, 0.0)),
            entry(3.6, new ShotParameter(20, 3000, 0.0)),
            entry(4.5, new ShotParameter(24, 3000, 0.0)),
            entry(5.4, new ShotParameter(25.5, 3200, 0.0)),
            entry(6.4, new ShotParameter(25, 3250, 0.0)),
            entry(7.46, new ShotParameter(27, 3400, 0.0)),
            entry(8.47, new ShotParameter(34, 3850, 0.0))
        )
    );

    // Method to get shot parameters based on vision distances
    public static ShotParameter get(double distance) {
        Entry<Double, ShotParameter> ceilEntry = map.ceilingEntry(distance);
        Entry<Double, ShotParameter> floorEntry = map.floorEntry(distance);
        if (ceilEntry == null) return floorEntry.getValue();
        if (floorEntry == null) return ceilEntry.getValue();
        if (ceilEntry.getValue().equals(floorEntry.getValue())) return ceilEntry.getValue();
        return ceilEntry.getValue().interpolate(
            floorEntry.getValue(), 
            (distance - floorEntry.getKey())/(ceilEntry.getKey() - floorEntry.getKey())
        );
    }

}