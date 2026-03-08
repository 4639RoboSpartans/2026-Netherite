/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.network;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

public class Network {
  // singleton
  private static volatile Network instance;

  static synchronized Network getInstance() {
    return Objects.requireNonNullElseGet(instance, () -> instance = new Network());
  }

  private final NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

  private final Map<String, Table> tableCache = new HashMap<>();

  public static Table getTable(String tableName) {
    return getInstance()
        .tableCache
        .computeIfAbsent(
            tableName,
            key -> new Table(tableName, getInstance().networkTableInstance.getTable(key)));
  }

  public static void printAllEntriesFromTable(String tableName) {
    getTable(tableName)
        .getAllEntries()
        .forEach((key, value) -> SmartDashboard.putString(tableName + ":" + key, value));
  }
}
