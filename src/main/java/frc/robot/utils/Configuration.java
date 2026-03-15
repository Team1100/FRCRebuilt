package frc.robot.utils;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.Scanner;
import java.util.HashMap;

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

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Robot;
import frc.robot.utils.vision.VisionConfig;

class MotorConfig {
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

  private void init() {
    String myIdentity = "";
    if (Robot.isSimulation()) {
      Scanner scanner = new Scanner(System.in);
      System.out.println("What identity do I have?");
      myIdentity = scanner.nextLine();
      scanner.close();
    } else {
      File f = new File(System.getenv("HOME") + "/identity");
      try (java.io.FileReader r = new java.io.FileReader(f)) {
        char[] cbuf = new char[32];
        r.read(cbuf, 0, 64);
        myIdentity = cbuf.toString();
      } catch (IOException e) {
        System.err.printf("Error finding identity file: %s\n%s", e.getMessage(), e.getStackTrace());
      }
    }

    if (myIdentity.isEmpty()) myIdentity = "default";
    
    File deployDir = Filesystem.getDeployDirectory();
    File[] configFiles = deployDir.listFiles((dir, name) -> {return name.equals("1100_config.json");});
    if (configFiles.length > 0 && configFiles[0].exists()) {
      File configFile = configFiles[0];

      ObjectMapper mapper = new ObjectMapper();
      try {
        JsonNode node = mapper.readTree(configFile);
        m_config = node.findValue(myIdentity);
      } catch (IOException e) {
        System.err.printf("Error reading config file: %s\n%s", e.getMessage(), e.getStackTrace());
      }

      if (m_config == null) {
        System.out.println("No config named " + myIdentity);
        return;
      }
    } else {
      System.out.println("No 1100_config.json found in deploy directory " + deployDir.getAbsolutePath());
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
    if (visionConfig != null) {
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
      for (int i = 0; i < subsystemValues.size(); i++) {
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
    if (values == null) {
      loadSubsystem(subsystem);
      values = m_values.get(subsystem);
    }
    if (values == null || values.get(name) == null) {
       System.err.println("Unconfigured double " + subsystem + "/" + name);
       return 0.0;
    }
    return Double.valueOf(values.get(name).toString()).doubleValue();
  }

  public int getInt(String subsystem, String name)
  {
    Map<String, Object> values = m_values.get(subsystem);
    if (values == null) {
      loadSubsystem(subsystem);
      values = m_values.get(subsystem);
    }
    if (values == null || values.get(name) == null) {
      System.err.println("Unconfigured integer " + subsystem + "/" + name);
      return 0;
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
    if (values == null || values.get(name) == null)
    {
      System.err.println("Unconfigured boolean " + subsystem + "/" + name);
      return false;
    }
    return Boolean.valueOf(values.get(name).toString()).booleanValue();
  }
}
