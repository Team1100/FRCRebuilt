package frc.robot.utils;

import java.io.BufferedReader;
import java.io.File;
import java.util.List;
import java.util.Map;
import java.util.HashMap;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.utils.vision.VisionConfig;

class MotorConfig
{
  @JsonCreator
  public MotorConfig(@JsonProperty(value="cType") String controllerType,
                     @JsonProperty(value="mType") MotorType motorType,
                     @JsonProperty(value="canid") int canid)
  {
    m_controllerType = controllerType;
    m_motorType = motorType;
    m_CANid = canid;
  }

  public String m_controllerType;
  public MotorType m_motorType;
  public int m_CANid;
}

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

public class Configuration {

  JsonNode m_config;
  List<VisionConfig> m_VisionConfigs;
  Map<String, MotorConfig> m_motorConfigs;
  Map<String, Map<String, Object>> m_values = new HashMap<String, Map<String, Object>>();
  static Configuration theInstance;
  synchronized public static Configuration getInstance()
  {
    if (theInstance == null)
    {
      theInstance = new Configuration();
      theInstance.init();
    }
    return theInstance;
  }

  private void init()
  {
    try {
      String home = java.lang.System.getenv("HOME");
      if (home == null || home.isEmpty()) {
        home = "/home/lvuser";
      }
      else {
        System.out.println("$HOME is " + home);
      }
      String myIdentity = "";
      File f = new File(home + "/identity");
      if (f.exists())
      {
        java.io.FileReader r = new java.io.FileReader(f);
        java.io.BufferedReader b = new BufferedReader(r);
        myIdentity = b.readLine();
      }
      if (myIdentity.isEmpty()) myIdentity = "default";
      System.out.println("Identity is " + myIdentity);
      String jsonfilepath = home + "/deploy/1100_config.json";
      File configfile = new File(jsonfilepath);
      if (configfile.exists())
      {
        ObjectMapper mapper = new ObjectMapper();
        JsonNode node = mapper.readTree(configfile);
        m_config = node.findValue(myIdentity);
        if (m_config == null)
        {
          System.out.println("no config named " + myIdentity);
          return;
        }
      }
      else
      {
        System.out.println("no json found at " + jsonfilepath);
      }
    } catch (Exception e) {
      System.err.println("exception from RobotMap.init():" + e.toString());
      // can't read the config. Carry on.
    }
  }

  public class ControllerAndConfig {
    public SparkBase m_controller;
    public SparkBaseConfig m_config;
    ControllerAndConfig() {
      m_controller = null;
      m_config = null;
    }
  }

  public ControllerAndConfig getMotorController(String motorName)
  {
    ControllerAndConfig rval = new ControllerAndConfig();
    if (m_motorConfigs == null)
    {
      JsonNode motorConfigs = m_config.findValue("motorConfigs");
      try {
        ObjectMapper om = new ObjectMapper();
        m_motorConfigs = om.readValue(motorConfigs.toString(), new TypeReference<Map<String, MotorConfig>>(){});
      } catch (Exception x) {
        System.out.println("getMotorConfig: oops, " + x);
        return rval;
      }
    }
    MotorConfig whichConfig = m_motorConfigs.get(motorName);
    if (whichConfig == null)
    {
      System.err.println("No motor configuration named " + motorName);
      return rval;
    }

    switch (whichConfig.m_controllerType)
    {
      case "SparkFlex":
        rval.m_controller = new SparkFlex(whichConfig.m_CANid, whichConfig.m_motorType);
        rval.m_config = new SparkFlexConfig();
        break;
      case "SparkMax":
        rval.m_controller = new SparkMax(whichConfig.m_CANid, whichConfig.m_motorType);
        rval.m_config = new SparkMaxConfig();
        System.out.println("creating SparkMax with id " + whichConfig.m_CANid);
        break;
      default:
        System.err.println("Invalid controller type " + whichConfig.m_controllerType);
    }
    return rval;
  }

  public List<VisionConfig> getVisionConfigs()
  {
 /*
    VisionConfig test = new VisionConfig("foo", new Translation3d(), new Rotation3d(), PoseStrategy.LOWEST_AMBIGUITY, PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT);
    ObjectMapper testOm = new ObjectMapper();
    try
    {
      testOm.writeValue(new File("/Users/markhecker/jsontest"), test);
    }
    catch (Exception x)
    {
      System.err.println("test oops: " + x);
    }
 */
    
    if (m_VisionConfigs != null) return m_VisionConfigs;
    if (m_config == null) return null;
    JsonNode visionConfig = m_config.findValue("visionConfig");
    if (visionConfig != null) 
    {
      System.out.println("got vision " + visionConfig.toString());
    }

    try {
      ObjectMapper om = new ObjectMapper();
      m_VisionConfigs = om.readValue(visionConfig.toString(), new TypeReference<List<VisionConfig>>(){});
    } catch (Exception x) {
      System.err.println("getVisionConfigs: oops, " + x);
    }
    System.out.println("visionConfigs has " + m_VisionConfigs.size());
    return m_VisionConfigs;
  }

  private void loadSubsystem(String subsystem)
  {
    m_values.put(subsystem, new java.util.HashMap<String, Object>());
    Map<String, Object> values = m_values.get(subsystem);

    JsonNode subsystemConfig = m_config.findValue(subsystem);
    try {
      ObjectMapper om = new ObjectMapper();
      List<RobotMapConfigValue> subsystemValues = om.readValue(subsystemConfig.toString(), new TypeReference<List<RobotMapConfigValue>>(){});
      for (int i = 0; i < subsystemValues.size(); i++)
      {
        RobotMapConfigValue value = subsystemValues.get(i);
        values.put(value.varName, value.value);
      }

    } catch (Exception x) {
      System.err.println("getSubsystemConfigs: oops, " + x);
    }
  }

  public double getDouble(String subsystem, String name)
  {
    Map<String, Object> values = m_values.get(subsystem);
    if (values == null)
    {
      loadSubsystem(subsystem);
      values = m_values.get(subsystem);
    }
    return Double.valueOf(values.get(name).toString()).doubleValue();
  }

  public int getInt(String subsystem, String name)
  {
    Map<String, Object> values = m_values.get(subsystem);
    if (values == null)
    {
      loadSubsystem(subsystem);
      values = m_values.get(subsystem);
    }
    return Integer.valueOf(values.get(name).toString()).intValue();
  }

  public boolean getBool(String subsystem, String name)
  {
    Map<String, Object> values = m_values.get(subsystem);
    if (values == null)
    {
      loadSubsystem(subsystem);
      values = m_values.get(subsystem);
    }
    return Boolean.valueOf(values.get(name).toString()).booleanValue();
  }
}
