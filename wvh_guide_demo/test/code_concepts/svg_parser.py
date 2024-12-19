import xml.etree.ElementTree as ET

def set_locations():
    graph = {}
    with open('/home/masettizan/ros2_ws/src/wvh_guide_demo/json/floors.geojson') as f:
        data = geojson.load(f)
        
    for feature in data['features']:
        info = {}
        info['x'] = float(feature['geometry']['coordinates'][0])
        info['y'] = float(feature['geometry']['coordinates'][1])
        info['neighbors'] = feature['properties']['neighbors']
        info['floor'] = feature['properties']['floor']
        graph[feature['properties']['id']] = info
    return graph

def element_to_dict(element,info):
    element_dict = {}
    element_dict['x'] = element.attrib.get('x')
    element_dict['y'] = element.attrib.get('y')
    element_dict['neighbors'] = element.attrib.get('neighbors')
    element_dict['floor'] = element.attrib.get('floor')
    
    if element.attrib.get('id') is not None:
        info[element.attrib.get('id')] = element_dict
    return info

def svg_to_dict(svg):
    tree = ET.parse(svg)
    root = tree.getroot()
    info = {}

    
    if list(root):
        for child in list(root):
            info = element_to_dict(child, info)

    return info

svg_file_path = '/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/WVH_1.svg'
svg_dict = svg_to_dict(svg_file_path)
print(svg_dict)