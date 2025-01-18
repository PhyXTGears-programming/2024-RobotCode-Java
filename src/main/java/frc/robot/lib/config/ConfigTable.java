package frc.robot.lib.config;

import java.util.Optional;

public interface ConfigTable {
    /**
     * Get double value from specified field.
     * @param String name - name of field in table (ex: "speed.min")
     * @return Optional in case config is missing a field with the specified name, or unable to convert string in config into a double.  Might need a Result type to indicate missing vs bad value.
     */
    Optional<double> getDouble(String name);
}
