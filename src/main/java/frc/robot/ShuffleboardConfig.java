// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;


/**
 * Singleton class for managing Shuffleboard dashboard. 
 * Provides support for the system dashboard layout.
 * Uses NetworkTable for subsystem communication.
 */
public class ShuffleboardConfig {
    // Create single instance
    private static ShuffleboardConfig instance = null;

    // Setup tabs for 'driver' and 'debug'
    public static String DriverTab = "Driver";
    public static String DebugTab = "Debug";
 

    private final ShuffleboardTab driverTab, debugTab;

    // Create tabs and layouts
    private ShuffleboardConfig() {
        driverTab = Shuffleboard.getTab(DriverTab);
        debugTab = Shuffleboard.getTab(DebugTab);
    }

    // getInstance to manage singleton
    public static ShuffleboardConfig getInstance() {
        if (instance == null) {
            instance = new ShuffleboardConfig();
        }

        return instance;
    }

    public static <T extends Enum<T>> GenericEntry putEntry(EnumMap<T, GenericEntry> map, T enumValue, Object defaultValue, 
        ShuffleboardContainer shuffleContainer, String name) {
            // use of var keyword enables the compiler to deduce the appropriate type
            var i = shuffleContainer.add(name, defaultValue)
                .withSize(2, 2)
                .withPosition(0, 0)
                .withProperties(Map.of("Label position", "Left"))
                .getEntry();
            map.put(enumValue, i);

            return i;
        }

    public void addAutoChooser(SendableChooser<Command> autoChooser) {
        driverTab.add("Autonomous Selector", autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2,1);
    }

    /** Setup dashboard for Roller Subsystem widgets
     *  It provides a centralized organized mechanism for building dashboard layout, 
     *  while giving the power to the subsystem on defining how their subsystem dashboard 
     *  is configured.
     */ 
    public static class RollerTabManager {
        // Reusing debug tab -- otherwise, specify a different tab name, ex. "Roller"
        public static String defaultTab = DebugTab;
        public static String RollerLayout = "Roller";
        //private static ShuffleboardLayout rollerOverview;

        private static ShuffleboardLayout buildLayout(String tabName) {
            ShuffleboardLayout rollerOverview = Shuffleboard
            .getTab(tabName)
            .getLayout(RollerLayout, BuiltInLayouts.kList)
            .withSize(3, 6)
            .withProperties(Map.of("Label position", "TOP"));

            return rollerOverview;
        }

        public static ShuffleboardTab getTab() {
            return Shuffleboard.getTab(defaultTab);
        }

        public static ShuffleboardLayout getLayout() {
            return buildLayout(defaultTab);
        }

        public static ShuffleboardLayout getLayout(String tabName) {
            return buildLayout(tabName);
        } 
    }
}