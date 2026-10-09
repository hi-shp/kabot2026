"""rosbag2 metadata compatibility, including latched static TF playback."""
import yaml
import rosbag2_py


def topic_metadata(topic, message_type, profiles):
    def duration(value):
        return dict(sec=value.nanoseconds//1000000000, nsec=value.nanoseconds % 1000000000)
    qos_yaml = yaml.safe_dump([dict(history=int(q.history), depth=q.depth,
        reliability=int(q.reliability), durability=int(q.durability),
        deadline=duration(q.deadline), lifespan=duration(q.lifespan),
        liveliness=int(q.liveliness), liveliness_lease_duration=duration(q.liveliness_lease_duration),
        avoid_ros_namespace_conventions=q.avoid_ros_namespace_conventions) for q in profiles])
    kwargs = dict(name=topic, type=message_type, serialization_format='cdr')
    try:
        # Humble uses serialized YAML and has no id parameter.
        return rosbag2_py.TopicMetadata(offered_qos_profiles=qos_yaml, **kwargs)
    except TypeError:
        from rosbag2_py import _storage
        # Modern rosbag2 stores C++ QoS objects; version 8 is the YAML format above.
        try:
            native_qos = _storage.to_rclcpp_qos_vector(qos_yaml, 8)
        except TypeError:
            native_qos = _storage.to_rclcpp_qos_vector(qos_yaml)
        return rosbag2_py.TopicMetadata(id=0, offered_qos_profiles=native_qos, **kwargs)
