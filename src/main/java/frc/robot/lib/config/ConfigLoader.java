package frc.robot.lib.config;

import java.util.Optional;

public interface ConfigLoader {
    void openConfig();

    /**
     * Allow user to access a specified table.
     * @param String name - name of table in config file.
     * @return Optional in case config is missing a table with the specified name.
     */
    Optional<ConfigTable> getTable(String name);
}
