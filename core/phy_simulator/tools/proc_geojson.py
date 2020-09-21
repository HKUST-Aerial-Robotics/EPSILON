import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection

import math
from pprint import pprint

# data_folder = '/home/denny/Dropbox/Dropbox/00-Research/IROS2019_wc_zl/qgis_playground/benchmark_track/geojson/'
# data_folder = '/home/zl/catkin_ws/src/vehicle_planning/qgis_projects/urban_google_prj/geojson/'
# data_folder = '/home/zl/catkin_ws/src/vehicle_planning/qgis_projects/highway/geojson/'
data_folder = '../../../toolchain/qgis_projects/highway/geojson/'

print(data_folder)

pt_feat_name = data_folder + "pt_feat" + ".geojson"
pt_feat_json_fs = open(pt_feat_name).read()
pt_feat_json = json.loads(pt_feat_json_fs)

lane_net_name = data_folder + "lane_net" + ".geojson"
lane_net_json_fs = open(lane_net_name).read()
lane_net_json = json.loads(lane_net_json_fs)

obstacles_name = data_folder + "obstacles" + ".geojson"
obstacles_json_fs = open(obstacles_name).read()
obstacles_json = json.loads(obstacles_json_fs)

for f in pt_feat_json["features"]:
  if f["properties"]["name"] == "origin":
    origin = np.array(f["geometry"]["coordinates"])

for f in lane_net_json["features"]:
  for pt in f["geometry"]["coordinates"][0]:
    pt[0] = pt[0] - origin[0]
    pt[1] = pt[1] - origin[1]

for f in obstacles_json["features"]:
  for pt in f["geometry"]["coordinates"][0][0]:
    pt[0] = pt[0] - origin[0]
    pt[1] = pt[1] - origin[1]

# save normed data
lane_net_norm_name = data_folder + "lane_net_norm" + ".json"
lane_net_norm_json_fs = open(lane_net_norm_name,'w')
json.dump(lane_net_json, lane_net_norm_json_fs)

obstacles_norm_name = data_folder + "obstacles_norm" + ".json"
obstacles_norm_json_fs = open(obstacles_norm_name,'w')
json.dump(obstacles_json, obstacles_norm_json_fs)

# plot map data
x_set = []
y_set = []

for f in lane_net_json["features"]:
  coord = np.array(f["geometry"]["coordinates"][0])
  x = coord[:,0]
  y = coord[:,1]

  x_set.append(x)
  y_set.append(y)

fig, ax = plt.subplots()

for i in range(len(x_set)):
  ax.plot(x_set[i],y_set[i],marker='o')

patches = []
for f in obstacles_json["features"]:
  coord = np.array(f["geometry"]["coordinates"][0][0])
  polygon = Polygon(np.array(coord), True)
  patches.append(polygon)

p = PatchCollection(patches, cmap=matplotlib.cm.jet, alpha=0.4)

colors = 100*np.random.rand(len(patches))
p.set_array(np.array(colors))
ax.add_collection(p)

plt.axis('equal')
plt.show()

