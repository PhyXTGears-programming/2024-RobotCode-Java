package frc.robot.lib.config;

import java.util.Optional;

import org.tomlj.TomlTable;

public class TomlConfigTable implements ConfigTable{
    private TomlTable mTable;

    public TomlConfigTable(TomlTable table) {
        mTable = table;
    }
    
    @Override
    public Optional<Boolean> getBoolean(String name) {
        return Optional.ofNullable(mTable.getBoolean(name));
    }

    @Override
    public Optional<Double> getDouble(String name) {
        return Optional.ofNullable(mTable.getDouble(name));
    }

    
}
