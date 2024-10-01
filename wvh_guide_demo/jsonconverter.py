#! /usr/bin/env python3
import geojson
'''
{"geometry": {
    "coordinates": [0.0, 0.0],
    "type": "Point"
    }, 
"properties": {
    "floor": 1, 
    "id": "f1_p1", 
    "neighbors": ["f1_p2"]
    }, 
"type": "Feature"
}
'''

'''
info = {}
info['x'] = float(row['position_x'])
info['y'] = float(row['position_y'])
info['neighbors'] = row['neighbors'].split(',')
self.graph[row['location']] = info
'''

with open('/home/masettizan/ros2_ws/src/wvh_guide_demo/json/floors.geojson') as f:
    data = geojson.load(f)
    
for feature in data['features']:
    info = {}
    info['x'] = float(feature['geometry']['coordinates'][0])
    info['y'] = float(feature['geometry']['coordinates'][1])
    info['neighbors'] = feature['properties']['neighbors']
    info['floor'] = feature['properties']['floor']
    print(info)
    # print (feature['geometry']['type'])
    # print (feature['geometry']['coordinates'])
