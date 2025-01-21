package frc.robot.lib.config;

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
        TomlParseResult result = Toml.parse(mConfigFile);
        result.errors().forEach(error -> System.err.println(error.toString()));
        mToml = result;
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
