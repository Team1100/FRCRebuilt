package frc.robot.utils.trajectory;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;

public class InterpolatingVelocityMap {
    private final TreeMap<Double, VelocityMapping> m_map;

    public InterpolatingVelocityMap() {
        m_map = new TreeMap<>();
    }

    public InterpolatingVelocityMap(Map<Double, VelocityMapping> map) {
        m_map = new TreeMap<>(map);
    }

    public VelocityMapping get(double angle) {
        if (m_map.size() == 0) return null;
        Double ceil = m_map.ceilingKey(angle);
        Double floor = m_map.floorKey(angle);
        if (floor == null) floor = ceil;
        if (ceil == null) ceil = floor;

        double t = MathUtil.inverseInterpolate(floor, ceil, angle);

        VelocityMapping r = VelocityMapping.interpolate(m_map.get(floor), m_map.get(ceil), t);
        return r;
    }

    public void put(double angle, VelocityMapping mapping) {
        m_map.put(angle, mapping);
    }
}
