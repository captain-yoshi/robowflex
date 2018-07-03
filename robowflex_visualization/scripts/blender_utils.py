#!/usr/bin/env python
''' A number of simple functions used in multiple different scripts. '''
# pylint: disable=import-error
import bpy


def add_mat_to_obj(obj, mat):
    ''' Adds a material to an object. '''
    if obj.data.materials:
        # assign to 1st material slot
        obj.data.materials[0] = mat
    else:
        # no slots
        obj.data.materials.append(mat)


def add_camera(location, rotation):
    ''' Adds camera to a location and a quaternion rotation. '''
    bpy.ops.object.camera_add(location=location)
    bpy.context.scene.camera = bpy.context.active_object
    bpy.context.active_object.rotation_mode = 'QUATERNION'
    bpy.context.active_object.rotation_quaternion = rotation


def add_sun(location, shadow):
    ''' Adds a sun/parallel light source to the scene. '''
    bpy.ops.object.lamp_add(type='SUN', location=location)
    if shadow:
        bpy.context.active_object.data.shadow_method = 'RAY_SHADOW'
        bpy.context.active_object.data.shadow_ray_samples = 6


def delete_all():
    '''
    Deletes all selectable things from the scene (objects, cameras, lights, etc)
    '''
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()


def hex_to_rgb(hex_str):
    ''' Converts a RGB hex string into three tuple of color. Supports anything like *123456'''
    # pylint: disable=maybe-no-member
    #return tuple([component / 255 for component in bytes.fromhex(hex_str[-6:])])

    gamma = 2.2
    value = hex_str.lstrip('#')
    lv = len(value)
    fin = list(int(value[i:i + lv // 3], 16) for i in range(0, lv, lv // 3))
    r = pow(fin[0] / 255, gamma)
    g = pow(fin[1] / 255, gamma)
    b = pow(fin[2] / 255, gamma)
    return (r, g, b)


def pose_to_quat(pose):
    '''
    Takes a pose dict and extracts the orientation quaternion.
    ROS quaternions or XYZW, but Blender's are WXYZ, so reorder them.
    '''
    return pose['orientation'][3:] + pose['orientation'][:3]


def pose_to_vec(pose):
    ''' Takes a pose dict and extracts the position vector. '''
    return pose['position']


def set_pose(obj, pose):
    ''' Sets the pose of a blender object by passing in a pose dict. '''
    obj.location = pose_to_vec(pose)
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = pose_to_quat(pose)
