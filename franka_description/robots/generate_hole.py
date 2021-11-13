# Taken from https://github.com/HarvardAgileRoboticsLab/gym-kuka-mujoco/blob/a01877d69b90f888df677e3aefa8ff3dd3598842/gym_kuka_mujoco/utils/gen_hole.py
# Adapted for URDF instead of mujoco xmls
import xml.etree.ElementTree as ET
import xml.dom.minidom
import numpy as np
import os

# ##################################################################
# A python script to generate a cylindrical hole by approximating 
# with multiple rotated boxes. Height determines the depth of this 
# cylindrical hole. Creates a link frame which is centered at the 
# bottom of the hole. z axis is along the hole centerline.
# ##################################################################

def add_attributes(xml_element, data):
    '''
    Adds attributed to the XML element from a dictionary.
    '''
    for key, val in data.items():
        if isinstance(val, str):
            new_val = val
        elif isinstance(val, tuple):
            new_val = " ".join([str(v) for v in val])
        xml_element.set(key, new_val)

def gen_hole(inner_radius, outer_radius, height, num_facets, fraction=1.0, connect_inner=False, radians=True):
    '''
    Generates a list of box geoms that approximate a hole.
    '''
    # Compute size.
    depth = outer_radius - inner_radius

    # Determine the width based on connecting the inner radius or the outer radius.
    if connect_inner:
        width = 2*np.sqrt((inner_radius/np.cos(np.pi/num_facets))**2 - inner_radius**2)
    else:
        width = 2*np.sqrt((outer_radius/np.cos(np.pi/num_facets))**2 - outer_radius**2)

    # Compute position.
    theta = np.arange(0,num_facets)*2*np.pi/num_facets
    theta *= fraction
    y_pos = np.sin(theta)*(inner_radius + depth/2)
    x_pos = np.cos(theta)*(inner_radius + depth/2)
    
    # Assemble the geoms.
    geoms = []
    for i in range(num_facets):
        geoms.append({
            'geometry':'box',
            # 'xyz':(x_pos[i], y_pos[i], height/2),
            'xyz':(x_pos[i], y_pos[i], height/2),
            'rpy':(0, 0, np.pi/2 + theta[i]),
            # 'size':(width/2, depth/2, height/2),
            'size':(width, depth, height),
        })
        # else:
        #     geoms.append({
        #         'type':'box',
        #         'pos':(x_pos[i], y_pos[i], height/2),
        #         'euler':(0, 0, 90+180.0/(np.pi)*theta[i]),
        #         'size':(width/2, depth/2, height/2),
        #         'class':'collision'
        #     })
    # geoms.append({
    #     'type':'cylinder',
    #     'pos': (0, 0, -height/2),
    #     'size': (outer_radius, height/2),
    #     'class': 'collision'
    # })
    return geoms

def write_xml(filename, element):
    '''
    Takes an xml.etree.ElementTree element and writes it to a human readable file.
    '''
    # Format and write the XML file.
    xml_string = ET.tostring(element)
    # import pdb; pdb.set_trace()
    xml_dom = xml.dom.minidom.parseString(xml_string)
    pretty_xml_string = xml_dom.toprettyxml()

    # os.makedirs(os.path.dirname(filename), exist_ok=True)
    dirname = os.path.dirname(filename)
    # print(dirname)

    if not os.path.exists(dirname):
        os.makedirs(dirname)

    with open(filename, 'w') as f:
        f.write(pretty_xml_string)


def assemble_urdf_xml(geoms, is_static=True):
    '''
    Assembles a list of geoms into XML that can be read by URDF
    '''
    # Create the XML structure
    robot = ET.Element('robot')
    add_attributes(robot, {'name':'hole', 'xmlns:xacro':'http://www.ros.org/wiki/xacro'})

    # Bottom of center of hole
    link = ET.SubElement(robot, 'link')
    add_attributes(link, {'name':'hole_link'})

    # Add arbitrary mass/inertia so gazebo doesn't complain
    inertial = ET.SubElement(link, 'inertial')
    mass = ET.SubElement(inertial, 'mass')
    mass.set('value', '1')
    inertia = ET.SubElement(inertial, 'inertia')
    add_attributes(inertia, {'ixx':'1', 'iyy':'1', 'izz':'1', 'ixy':'0', 'ixz':'0', 'iyz':'0',})




    # Adds all of the geoms to the collision and visual elems

    for g in geoms:
        # Add collision geometry
        collision = ET.SubElement(link, 'collision')
        coll_origin = ET.SubElement(collision, 'origin')
        # add_attributes(coll_origin, g['xyz'])
        # add_attributes(coll_origin, g['rpy'])
        coll_origin.set('xyz', " ".join([str(v) for v in g['xyz']]))
        coll_origin.set('rpy', " ".join([str(v) for v in g['rpy']]))
        coll_geom = ET.SubElement(collision, 'geometry')
        coll_shape = ET.SubElement(coll_geom, g['geometry'])
        # add_attributes(coll_shape, g['size'])
        coll_shape.set('size', " ".join([str(v) for v in g['size']]))

        # # Add visual geometry
        visual = ET.SubElement(link, 'visual')
        vis_origin = ET.SubElement(visual, 'origin')
        # add_attributes(visual_origin, g['xyz'])
        # add_attributes(visual_origin, g['rpy'])
        vis_origin.set('xyz', " ".join([str(v) for v in g['xyz']]))
        vis_origin.set('rpy', " ".join([str(v) for v in g['rpy']]))
        vis_geom = ET.SubElement(visual, 'geometry')
        vis_shape = ET.SubElement(vis_geom, g['geometry'])
        # add_attributes(vis_shape, g['size'])
        vis_shape.set('size', " ".join([str(v) for v in g['size']]))
    
    # Gazebo specific tags
    gazebo = ET.SubElement(robot, 'gazebo')

    material = ET.SubElement(gazebo, 'material')
    material.text = 'Gazebo/gray'

    static = ET.SubElement(gazebo, 'static')
    static.text = 'true'

    return robot

if __name__ == "__main__":
    # Generate hole sized for the peg on the robot
    # 0.0127 is radius of peg
    # 0.111125 is length
    hole_geoms = gen_hole(0.0135, .03, .12, 16) #inner radius, outer radius, height, num of facets 
    hole_xml = assemble_urdf_xml(hole_geoms, True)    
    filename = 'hole.urdf'
    # model_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'holes', filename)
    model_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), filename)
    # print(model_path)
    write_xml(model_path, hole_xml)

    # # Generate and save a huge hole.
    # hole_geoms = gen_hole(0.04, .05, .05, 16)
    # hole_sites = [
    #     {'name':'hole_base', 'pos':(0,0,0), 'euler':(-np.pi,0,0), 'size':(0.005, 0.005, 0.005)},
    #     {'name':'hole_top', 'pos':(0,0,0.05), 'euler':(-np.pi,0,0), 'size':(0.005, 0.005, 0.005)}
    # ]
    # hole_xml = assemble_mujoco_xml(hole_geoms, hole_sites)
    # filename = 'polyhedral_hole_inner=0-040_outer=0-05_height=0-05_num_facets=16.xml'
    # model_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'envs', 'assets', 'hole', filename)
    # write_xml(model_path, hole_xml)

    # # Generate and save a big hole.
    # hole_geoms = gen_hole(0.008, .05, .05, 16)
    # hole_sites = [
    #     {'name':'hole_base', 'pos':(0,0,0), 'euler':(-np.pi,0,0), 'size':(0.005, 0.005, 0.005)},
    #     {'name':'hole_top', 'pos':(0,0,0.05), 'euler':(-np.pi,0,0), 'size':(0.005, 0.005, 0.005)}
    # ]
    # hole_xml = assemble_mujoco_xml(hole_geoms, hole_sites)
    # filename = 'polyhedral_hole_inner=0-008_outer=0-05_height=0-05_num_facets=16.xml'
    # model_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'envs', 'assets', 'hole', filename)
    # write_xml(model_path, hole_xml)

    # # Generate and save a medium hole.
    # hole_geoms = gen_hole(0.0075, .05, .05, 16)
    # hole_sites = [
    #     {'name':'hole_base', 'pos':(0,0,0), 'euler':(-np.pi,0,0), 'size':(0.005, 0.005, 0.005)},
    #     {'name':'hole_top', 'pos':(0,0,0.05), 'euler':(-np.pi,0,0), 'size':(0.005, 0.005, 0.005)}
    # ]
    # hole_xml = assemble_mujoco_xml(hole_geoms, hole_sites)
    # filename = 'polyhedral_hole_inner=0-0075_outer=0-05_height=0-05_num_facets=16.xml'
    # model_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'envs', 'assets', 'hole', filename)
    # write_xml(model_path, hole_xml)

    # # Generate and save a small hole.
    # hole_geoms = gen_hole(0.0070, .05, .05, 16)
    # hole_sites = [
    #     {'name':'hole_base', 'pos':(0,0,0), 'euler':(-np.pi,0,0), 'size':(0.005, 0.005, 0.005)},
    #     {'name':'hole_top', 'pos':(0,0,0.05), 'euler':(-np.pi,0,0), 'size':(0.005, 0.005, 0.005)}
    # ]
    # hole_xml = assemble_mujoco_xml(hole_geoms, hole_sites)
    # filename = 'polyhedral_hole_inner=0-0070_outer=0-05_height=0-05_num_facets=16.xml'
    # model_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'envs', 'assets', 'hole', filename)
    # write_xml(model_path, hole_xml)

    # # Generate and save a tiny hole.
    # hole_geoms = gen_hole(0.00685, .05, .05, 16)
    # hole_sites = [
    #     {'name':'hole_base', 'pos':(0,0,0), 'euler':(-np.pi,0,0), 'size':(0.005, 0.005, 0.005)},
    #     {'name':'hole_top', 'pos':(0,0,0.05), 'euler':(-np.pi,0,0), 'size':(0.005, 0.005, 0.005)}
    # ]
    # hole_xml = assemble_mujoco_xml(hole_geoms, hole_sites)
    # filename = 'polyhedral_hole_inner=0-00685_outer=0-05_height=0-05_num_facets=16.xml'
    # model_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'envs', 'assets', 'hole', filename)
    # write_xml(model_path, hole_xml)

    # # Generate and save many holes
    # upper = 0.04
    # lower = 0.0068
    # minimum = 0.006725
    # inner_sizes = np.logspace(np.log10(upper-minimum),np.log10(lower-minimum),100) + minimum

    # for i,s in enumerate(inner_sizes):
    #     hole_geoms = gen_hole(s, .10, .05, 16)
    #     hole_sites = [
    #         {'name':'hole_base', 'pos':(0,0,0), 'euler':(-np.pi,0,0), 'size':(0.005, 0.005, 0.005)},
    #         {'name':'hole_top', 'pos':(0,0,0.05), 'euler':(-np.pi,0,0), 'size':(0.005, 0.005, 0.005)}
    #     ]
    #     hole_xml = assemble_mujoco_xml(hole_geoms, hole_sites)
    #     filename = 'polyhedral_hole_inner=0-{:06d}_outer=0-10_height=0-05_num_facets=16_id={:03d}.xml'.format(int(np.round(s*1000000)), i)
    #     print(filename)
    #     model_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'envs', 'assets', 'hole', filename)
    #     write_xml(model_path, hole_xml)