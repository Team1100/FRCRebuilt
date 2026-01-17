// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.lang.reflect.Field;
import java.util.List;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

/*
  JSON RobotMap config format:
  [
    {
      "VarName": "NAME_OF_VARIABLE",
      "Value": NEW_VALUE_OF_CORRESPONDING_TYPE
    }
  ]
*/

class RobotMapConfigValue {
  String varName = null;
  Object value = null;

  public void setVarName(String varName) {
      this.varName = varName;
  }

  public void setValue(Object value) {
      this.value = value;
  }
}

/** Add your docs here. */
public class RobotMap {

  public static void init()
  {
    try {
      String home = java.lang.System.getenv("HOME");
      if (home == null || home.isEmpty()) {
        home = "/home/lvuser";
      }
      else {
        System.out.println("$HOME is " + home);
      }
      File f = new File(home + "/1100_Config");
      if (f.exists())
      {
        ObjectMapper mapper = new ObjectMapper();
        List<RobotMapConfigValue> changedValues = mapper.readValue(f, new TypeReference<List<RobotMapConfigValue>>(){});

        for (int i=0; i < changedValues.size(); i++) {
          RobotMapConfigValue value = changedValues.get(i);
          try {
            Field field = RobotMap.class.getField(value.varName);
            field.set(null, value.value);
            System.out.println("Configuring " + value.varName + " to " + value.value);
          }
          catch (java.lang.NoSuchFieldException x) {
            System.out.println("No RobotMap field named " + value.varName);
          }
        }
      }  
      else
      {
        System.out.println("No configuration present, using defaults");
      }          
    } catch (Exception e) {
      System.err.println("exception from RobotMap.init():" + e.toString());
      // can't read the config. Carry on.
    }

  }

  // [R]obot Geometry
  // TODO: get radius
  public static double R_BASE_RADIUS_INCHES = 0;

  // [U]ser Input
  public static int U_DRIVER_XBOX_CONTROLLER = 0;
  public static int U_OPERATOR_XBOX_CONTROLLER = 1;

  // [D]rive
  // TODO: fill in drive mappings
  public static int D_FRONT_RIGHT_DRIVE = RoboRioMap.CAN_0;
  public static int D_FRONT_LEFT_DRIVE = RoboRioMap.CAN_0;
  public static int D_BACK_RIGHT_DRIVE = RoboRioMap.CAN_0;
  public static int D_BACK_LEFT_DRIVE = RoboRioMap.CAN_0;
  // TODO: fill in turning mappings
  public static int D_FRONT_RIGHT_TURNING = RoboRioMap.CAN_0;
  public static int D_FRONT_LEFT_TURNING = RoboRioMap.CAN_0;
  public static int D_BACK_RIGHT_TURNING = RoboRioMap.CAN_0;
  public static int D_BACK_LEFT_TURNING = RoboRioMap.CAN_0;

  // [I]ntake 
  public static boolean I_ENABLED = true;

  // [S]hooter
  public static boolean S_ENABLED = true;

  // [V]ision
  public static boolean V_ENABLED = true;
}