#!/usr/bin/env python3
"""
Script to fix URDF coordinate frame by applying 180° rotation around Z-axis.
This transforms all xyz coordinates and rpy orientations in the URDF.
"""

import xml.etree.ElementTree as ET
import math
import sys

def rotate_180_z(x, y, z):
    """Apply 180° rotation around Z-axis to xyz coordinates"""
    return -x, -y, z

def rotate_rpy_180_z(r, p, y):
    """Apply 180° rotation around Z-axis to rpy orientation"""
    # Adding π to yaw and normalizing
    new_yaw = y + math.pi
    # Normalize to [-π, π]
    while new_yaw > math.pi:
        new_yaw -= 2 * math.pi
    while new_yaw < -math.pi:
        new_yaw += 2 * math.pi
    return r, p, new_yaw

def process_origin(origin_elem):
    """Process an origin element and transform its xyz and rpy"""
    if origin_elem is None:
        return

    # Process xyz
    xyz_str = origin_elem.get('xyz', '0 0 0')
    xyz = [float(v) for v in xyz_str.split()]
    if len(xyz) == 3:
        new_xyz = rotate_180_z(xyz[0], xyz[1], xyz[2])
        origin_elem.set('xyz', f'{new_xyz[0]} {new_xyz[1]} {new_xyz[2]}')

    # Process rpy
    rpy_str = origin_elem.get('rpy', '0 0 0')
    rpy = [float(v) for v in rpy_str.split()]
    if len(rpy) == 3:
        new_rpy = rotate_rpy_180_z(rpy[0], rpy[1], rpy[2])
        origin_elem.set('rpy', f'{new_rpy[0]} {new_rpy[1]} {new_rpy[2]}')

def fix_urdf(input_file, output_file):
    """Fix URDF coordinate frame"""
    # Parse XML
    tree = ET.parse(input_file)
    root = tree.getroot()

    # Find the macro element
    macro = root.find('.//{http://www.ros.org/wiki/xacro}macro')
    if macro is None:
        print("Error: Could not find xacro:macro element")
        return False

    # Remove base_footprint link and joint if they exist
    for link in macro.findall('.//{http://www.ros.org/wiki/xacro}link'):
        if link.get('name') == 'base_footprint':
            macro.remove(link)

    for joint in macro.findall('.//{http://www.ros.org/wiki/xacro}joint'):
        if joint.get('name') == 'base_footprint_joint':
            macro.remove(joint)

    # Also check without namespace
    for link in macro.findall('.//link'):
        if link.get('name') == 'base_footprint':
            macro.remove(link)

    for joint in macro.findall('.//joint'):
        if joint.get('name') == 'base_footprint_joint':
            macro.remove(joint)

    # Process all inertial origins
    for inertial in macro.findall('.//inertial/origin'):
        process_origin(inertial)

    # Process all joint origins
    for joint in macro.findall('.//joint/origin'):
        process_origin(joint)

    # Process all visual origins
    for visual in macro.findall('.//visual/origin'):
        process_origin(visual)

    # Process all collision origins
    for collision in macro.findall('.//collision/origin'):
        process_origin(collision)

    # Write output
    tree.write(output_file, encoding='utf-8', xml_declaration=True)
    print(f"✓ Fixed URDF written to: {output_file}")
    return True

if __name__ == '__main__':
    input_file = 'src/description/urdf/athena_drive_description.urdf.xacro'
    output_file = 'src/description/urdf/athena_drive_description.urdf.xacro.fixed'

    if len(sys.argv) > 1:
        input_file = sys.argv[1]
    if len(sys.argv) > 2:
        output_file = sys.argv[2]

    print(f"Processing: {input_file}")
    if fix_urdf(input_file, output_file):
        print(f"\nNext steps:")
        print(f"1. Review the changes: diff {input_file} {output_file}")
        print(f"2. If satisfied, backup and replace: ")
        print(f"   cp {input_file} {input_file}.backup")
        print(f"   mv {output_file} {input_file}")
