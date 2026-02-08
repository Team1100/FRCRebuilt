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

  // [R]obot Geometry
  // TODO: get radius
  public static double R_BASE_RADIUS_INCHES = 19;

  // [U]ser Input
  public static int U_DRIVER_XBOX_CONTROLLER = 0;
  public static int U_OPERATOR_XBOX_CONTROLLER = 1;

  // [S]hooter
  public static boolean S_ENABLED = true;

  // [V]ision
  public static boolean V_ENABLED = true;

  // [C]limber
  public static boolean C_ENABLED = true;
}