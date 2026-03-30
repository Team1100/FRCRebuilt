package frc.robot.utils.structlogging;

import java.nio.ByteBuffer;

import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.protobuf.Protobuf;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.testingdashboard.SubsystemBase;
import us.hebi.quickbuf.ProtoMessage;

public class StructLogger implements AutoCloseable {
    private static enum SendType {
        STRUCT,
        PROTOBUF,
        STRUCT_ARRAY,
    }

    private final SendType m_sendType;

    private final NetworkTableInstance m_ntable;

    private String m_name;

    private DataLog m_dataLog;
    private int m_dataLogEntry;
    private boolean m_print;

    private final StructPublisher<StructSerializable> m_structPublisher;
    private final Struct<StructSerializable> m_struct;
    private StructSerializable m_structLastValue;
    private StructSerializable m_structCurrentValue;

    private final StructArrayPublisher<StructSerializable> m_structArrayPublisher;
    private StructSerializable[] m_structArrayLastValue;
    private StructSerializable[] m_structArrayCurrentValue;

    private final ProtobufPublisher<ProtobufSerializable> m_protobufPublisher;
    private final Protobuf<ProtobufSerializable, ProtoMessage<?>> m_protobuf;
    private ProtobufSerializable m_protobufLastValue;
    private ProtobufSerializable m_protobufCurrentValue;

    private StructLogger(SubsystemBase subsystem, String name, Struct<StructSerializable> struct, StructSerializable value) {
        m_sendType = SendType.STRUCT;

        m_name = name;

        assert struct.getTypeClass().equals(value.getClass());

        m_struct = struct;

        m_ntable = NetworkTableInstance.getDefault();
        m_structPublisher = m_ntable.getStructTopic(m_name, m_struct).publish();
        m_structCurrentValue = value;
        m_structLastValue = null;

        m_protobufPublisher = null;
        m_protobuf = null;
        m_protobufCurrentValue = null;
        m_protobufLastValue = null;

        m_structArrayPublisher = null;
        m_structArrayCurrentValue = null;
        m_structArrayLastValue = null;

        m_print = false;

        DataLogManager.logNetworkTables(false);
        DataLogManager.start();
        m_dataLog = DataLogManager.getLog();
        m_dataLog.addSchema(m_struct);
        m_dataLogEntry = m_dataLog.start(m_name, m_struct.getTypeString());

        subsystem.registerStructLogger(this);
    }

    private StructLogger(SubsystemBase subsystem, String name, Protobuf<ProtobufSerializable, ProtoMessage<?>> protobuf, ProtobufSerializable value) {
        m_sendType = SendType.STRUCT;

        m_name = name;

        assert protobuf.getTypeClass().equals(value.getClass());

        m_protobuf = protobuf;

        m_ntable = NetworkTableInstance.getDefault();
        m_protobufPublisher = m_ntable.getProtobufTopic(m_name, m_protobuf).publish();
        m_protobufCurrentValue = value;
        m_protobufLastValue = null;

        m_structPublisher = null;
        m_struct = null;
        m_structCurrentValue = null;
        m_structLastValue = null;

        m_structArrayPublisher = null;
        m_structArrayCurrentValue = null;
        m_structArrayLastValue = null;

        m_print = false;

        DataLogManager.start();
        m_dataLog = DataLogManager.getLog();
        m_dataLog.addSchema(m_protobuf);
        m_dataLogEntry = m_dataLog.start(m_name, m_protobuf.getTypeString());

        subsystem.registerStructLogger(this);
    }

    private StructLogger(SubsystemBase subsystem, String name, Struct<StructSerializable> struct, StructSerializable[] value) {
        m_sendType = SendType.STRUCT_ARRAY;

        m_name = name;

        assert struct.getTypeClass().equals(value.getClass());

        m_struct = struct;

        m_ntable = NetworkTableInstance.getDefault();
        m_structArrayPublisher = m_ntable.getStructArrayTopic(m_name, m_struct).publish();
        m_structArrayCurrentValue = value;
        m_structArrayLastValue = null;

        m_protobufPublisher = null;
        m_protobuf = null;
        m_protobufCurrentValue = null;
        m_protobufLastValue = null;

        m_structPublisher = null;
        m_structCurrentValue = null;
        m_structLastValue = null;

        m_print = false;

        DataLogManager.logNetworkTables(false);
        DataLogManager.start();
        m_dataLog = DataLogManager.getLog();
        m_dataLog.addSchema(m_struct);
        m_dataLogEntry = m_dataLog.start(m_name, m_struct.getTypeString());

        subsystem.registerStructLogger(this);
    }

    public void close() {
        m_dataLog.finish(m_dataLogEntry);
    }

    private void postStruct() {
        if (m_structCurrentValue == null || m_structCurrentValue.equals(m_structLastValue)) return;

        m_structLastValue = m_structCurrentValue;
        m_structPublisher.set(m_structCurrentValue);

        double time = Timer.getFPGATimestamp();
        if (m_print) System.out.printf("%f: %s", time, m_structCurrentValue.toString());

        if (m_dataLog != null) {
            ByteBuffer bb = ByteBuffer.allocate(m_struct.getSize());
            m_struct.pack(bb, m_structCurrentValue);
            m_dataLog.appendRaw(m_dataLogEntry, bb, 0);
        }
    }

    private void postProtobuf() {
        if (m_protobufCurrentValue == null || m_protobufCurrentValue.equals(m_protobufLastValue)) return;

        m_protobufLastValue = m_protobufCurrentValue;
        m_protobufPublisher.set(m_protobufCurrentValue);

        double time = Timer.getFPGATimestamp();
        if (m_print) System.out.printf("%f: %s", time, m_protobufCurrentValue.toString());

        if (m_dataLog != null) {
            ProtoMessage<?> message = m_protobuf.createMessage();
            m_protobuf.pack(message, m_protobufCurrentValue);
            m_dataLog.appendRaw(m_dataLogEntry, message.toByteArray(), 0);
        }
    }

    private void postStructArray() {
        if (m_structArrayCurrentValue == null || m_structArrayCurrentValue.equals(m_structArrayLastValue)) return;

        m_structArrayLastValue = m_structArrayCurrentValue;
        m_structArrayPublisher.set(m_structArrayCurrentValue);

        double time = Timer.getFPGATimestamp();
        if (m_print) System.out.printf("%f: %s", time, m_structArrayCurrentValue.toString());

        if (m_dataLog != null) {
            ByteBuffer bb = ByteBuffer.allocate(m_struct.getSize() * m_structArrayCurrentValue.length);
            Struct.packArray(bb, m_structArrayCurrentValue, m_struct);
            m_dataLog.appendRaw(m_dataLogEntry, bb, 0);
        }
    }

    public void post() {
        switch (m_sendType) {
        case STRUCT:
            postStruct();
            break;
        case PROTOBUF:
            postProtobuf();
            break;
        case STRUCT_ARRAY:
            postStructArray();
            break;
        }
    }

    public void setStruct(StructSerializable value) {
        if (m_structCurrentValue == null || m_structCurrentValue.getClass().isAssignableFrom(value.getClass()))
            m_structCurrentValue = value;
    }

    public void setProtobuf(ProtobufSerializable value) {
        if (m_protobufCurrentValue == null || m_protobufCurrentValue.getClass().isAssignableFrom(value.getClass()))
            m_protobufCurrentValue = value;
    }

    public void setStructArray(StructSerializable value[]) {
        if (m_structArrayCurrentValue == null || m_structArrayCurrentValue.getClass().isAssignableFrom(value.getClass()))
            m_structArrayCurrentValue = value;
    }

    public StructSerializable getStruct() {
        return m_structCurrentValue;
    }

    public ProtobufSerializable getProtobuf() {
        return m_protobufCurrentValue;
    }

    public StructSerializable[] getStructArray() {
        return m_structArrayCurrentValue;
    }

    public void setPrint(boolean print) {
        m_print = print;
    }

    /* 
     * This is terrible; remind me to rewrite StructLogger later so it doesn't suck.
     * Java gets confused with generics (of course, who'd expect it to be good at OOP)
     * so I confuse it hard enough to let me off with a warning.
     */ 

    @SuppressWarnings("unchecked")
    public static StructLogger pose2dLogger(SubsystemBase subsystem, String name, Pose2d defaultValue) {
        Struct<Pose2d> a = Pose2d.struct;
        return new StructLogger(subsystem, name, a.getClass().cast(a), defaultValue);
    }

    @SuppressWarnings("unchecked")
    public static StructLogger pose3dLogger(SubsystemBase subsystem, String name, Pose3d defaultValue) {
        Struct<Pose3d> a = Pose3d.struct;
        return new StructLogger(subsystem, name, a.getClass().cast(a), defaultValue);
    }

    @SuppressWarnings("unchecked")
    public static StructLogger pose2dArrayLogger(SubsystemBase subsystem, String name, Pose2d[] defaultValues) {
        Struct<Pose2d> a = Pose2d.struct;
        return new StructLogger(subsystem, name, a.getClass().cast(a), defaultValues);
    }

    @SuppressWarnings("unchecked")
    public static StructLogger pose3dArrayLogger(SubsystemBase subsystem, String name, Pose3d[] defaultValues) {
        Struct<Pose3d> a = Pose3d.struct;
        return new StructLogger(subsystem, name, a.getClass().cast(a), defaultValues);
    }

    @SuppressWarnings("unchecked")
    public static StructLogger translation3dLogger(SubsystemBase subsystem, String name, Translation3d defaultValue) {
        Struct<Translation3d> a = Translation3d.struct;
        return new StructLogger(subsystem, name, a.getClass().cast(a), defaultValue);
    }

    @SuppressWarnings("unchecked")
    public static StructLogger translation3dArrayLogger(SubsystemBase subsystem, String name, Translation3d[] defaultValues) {
        Struct<Translation3d> a = Translation3d.struct;
        return new StructLogger(subsystem, name, a.getClass().cast(a), defaultValues);
    }

    @SuppressWarnings("unchecked")
    public static StructLogger photonPipelineResultLogger(SubsystemBase subsystem, String name, PhotonPipelineResult defaultValue) {
        Protobuf<PhotonPipelineResult, Photon.ProtobufPhotonPipelineResult> a = PhotonPipelineResult.proto;
        return new StructLogger(subsystem, name, a.getClass().cast(a), defaultValue);
    }
}