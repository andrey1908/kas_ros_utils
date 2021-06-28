import rosbag
from geometry_msgs.msg import TransformStamped, Transform
from urdf_parser_py.urdf import URDF, Robot
from transforms3d.euler import euler2quat


def fill_tf_buffer_with_static_transforms_from_bag(bag, tf_buffer):
    for topic, msg, t in bag.read_messages(topics=['/tf_static']):
        for transform in msg.transforms:
            tf_buffer.set_transform_static(transform, 'default_authority')
            

def fill_tf_buffer_with_static_transforms_from_urdf(urdf, tf_buffer):
    for joint in urdf.joints:
        if joint.type != 'fixed':
            continue
        transform = Transform()
        transform.translation.x, transform.translation.y, transform.translation.z = joint.origin.xyz
        transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z = euler2quat(*joint.origin.rpy, axes='sxyz')
        transform_stamped = TransformStamped(transform=transform)
        transform_stamped.header.frame_id = joint.parent
        transform_stamped.child_frame_id = joint.child
        tf_buffer.set_transform_static(transform_stamped, 'default_authority')


def fill_tf_buffer_with_static_transforms(transforms_source, tf_buffer):
    if isinstance(transforms_source, rosbag.bag.Bag):
        fill_tf_buffer_with_static_transforms_from_bag(transforms_source, tf_buffer)
    elif isinstance(transforms_source, Robot):
        fill_tf_buffer_with_static_transforms_from_urdf(transforms_source, tf_buffer)
    else:
        raise(RuntimeError)


def fill_tf_buffer_with_static_transforms_from_bag_file(rosbag_file, tf_buffer):
    with rosbag.Bag(rosbag_file) as bag:
        fill_tf_buffer_with_static_transforms_from_bag(bag, tf_buffer)


def fill_tf_buffer_with_static_transforms_from_urdf_file(urdf_file, tf_buffer):
    urdf = URDF.from_xml_file(urdf_file)
    fill_tf_buffer_with_static_transforms_from_urdf(urdf, tf_buffer)


def fill_tf_buffer_with_static_transforms_from_file(transforms_source_file, tf_buffer):
    if transforms_source_file.endswith('.bag'):
        fill_tf_buffer_with_static_transforms_from_bag_file(transforms_source_file, tf_buffer)
    elif transforms_source_file.endswith('.urdf'):
        fill_tf_buffer_with_static_transforms_from_urdf_file(transforms_source_file, tf_buffer)
    else:
        raise(RuntimeError)
