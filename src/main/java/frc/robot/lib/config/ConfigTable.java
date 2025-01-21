package frc.robot.lib.config;

import java.util.Optional;

public interface ConfigTable {
    Optional<Double> getDouble(String name);
}
