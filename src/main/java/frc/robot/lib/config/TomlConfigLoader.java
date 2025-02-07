package frc.robot.lib.config;

import java.io.IOError;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Optional;
import org.tomlj.Toml;
import org.tomlj.TomlParseResult;
import org.tomlj.TomlTable;

public class TomlConfigLoader implements ConfigLoader {

    private String mConfigFile;
    private TomlParseResult mToml;

    public TomlConfigLoader(String configFile) {
        mConfigFile = configFile;
    }

    @Override
    public void openConfig() {
        try {
            String file = Files.readString(Path.of(mConfigFile));
            TomlParseResult result = Toml.parse(file);
            result.errors().forEach(error -> System.err.println(error.toString()));
            mToml = result;
        } catch (Exception ex) {
            System.err.println("Error: unable to read config.toml at " + mConfigFile);
            System.exit(1);
        }
    }

    @Override
    public Optional<ConfigTable> getTable(String name) {
        TomlTable table = mToml.getTable(name);
        if (null == table) {
            return Optional.empty();
        } else {
            return Optional.of(new TomlConfigTable(table));
        }
    }
    
}
