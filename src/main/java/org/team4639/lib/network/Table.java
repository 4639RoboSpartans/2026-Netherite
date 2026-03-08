/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.network;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

// wraps a NetworkTable
public class Table {
  final String name;
  final NetworkTable table;

  final HashMap<String, NetworkTableEntry> cache = new HashMap<String, NetworkTableEntry>();

  Table(String name, NetworkTable table) {
    this.name = name;
    this.table = table;
  }

  public String getName() {
    return name;
  }

  public NetworkTableEntry getEntry(String key) {
    return cache.containsKey(key) ? cache.get(key) : cache.put(key, table.getEntry(key));
  }

  public boolean getBoolean(String key) {
    return getEntry(key).getBoolean(false);
  }

  public double getDouble(String key) {
    return getEntry(key).getDouble(0);
  }

  public float getFloat(String key) {
    return getEntry(key).getFloat(0.0f);
  }

  public int getInteger(String key) {
    return (int) getEntry(key).getInteger(0);
  }

  public String getString(String key) {
    return getEntry(key).getString("");
  }

  public double[] getDoubleArray(String key) {
    return getEntry(key).getDoubleArray(new double[] {});
  }

  public Map<String, String> getAllEntries() {
    Map<String, String> map = new HashMap<String, String>();
    table.getKeys().parallelStream()
        .forEach(
            key -> {
              var entry = getEntry(key);
              map.put(
                  key,
                  switch (entry.getType()) {
                    case kBoolean -> Boolean.toString(entry.getBoolean(false));
                    case kDouble -> Double.toString(entry.getDouble(0));
                    case kFloat -> Float.toString(entry.getFloat(0));
                    case kString -> entry.getString("UNKNOWN STRING");
                    case kDoubleArray -> Arrays.toString(entry.getDoubleArray(new double[] {}));
                    default -> "UNKNOWN VALUE TYPE: " + entry.getType().toString();
                  });
            });
    return map;
  }
}
