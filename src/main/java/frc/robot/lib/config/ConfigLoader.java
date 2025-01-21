package frc.robot.lib.config;

import java.util.Optional;

public interface ConfigLoader {
    void openConfig();

    Optional<ConfigTable> getTable(String name);
}
