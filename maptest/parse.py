from OSMPythonTools.overpass import Overpass, overpassQueryBuilder
from OSMPythonTools.data import Data
from collections import OrderedDict
import pickle
import math

from tkinter import *
from tkinter import ttk

COORDS = 40.4429814, -79.9413792
ZOOM = 15

# ll: lat/lon in degrees
# wmc: web mercator coordinate, x,y from 0-1
# tile: xyz tile index, x,y from 0 to 2^z-1

def ll_to_wmc(lat, lon):
	lat = math.radians(lat)
	lon = math.radians(lon)

	x = (1 / (2 * math.pi)) * (math.pi + lon)
	y = (1 / (2 * math.pi)) * (math.pi - math.log(math.tan((math.pi / 4) + lat / 2.0)))
	return x, y

def wmc_to_tile(x, y, z):
	return math.floor(x * 2**z), math.floor(y * 2**z)

def ll_to_tile(lat, lon, z):
	return wmc_to_tile(*ll_to_wmc(lat, lon), z)

def tile_to_wmc(x, y, z):
	return x / 2**z, y / 2**z

def tile_bounds_wmc(x, y, z):
	return *tile_to_wmc(x, y, z), *tile_to_wmc(x+1, y+1, z)

def tile_bounds_ll(x, y, z):
	west, north, east, south = tile_bounds_wmc(x, y, z)
	return *wmc_to_ll(west, north), *wmc_to_ll(east, south)

def ll_bounds_ll(lat, lon, z):
	return tile_bounds_ll(*ll_to_tile(lat, lon, z), z)

def wmc_to_ll(x, y):
	lat = 2.0 * (math.atan(math.exp(math.pi - 2 * math.pi * y)) - math.pi / 4)
	lon = 2.0 * math.pi * x - math.pi

	lat = math.degrees(lat)
	lon = math.degrees(lon)

	return lat, lon

query_template = """
(
  way[highway~"primary|secondary|tertiary|residential|service|unclassified|steps"]({bbox})(if: length() > 5);
  >;
  way[highway=footway][footway!~"sidewalk|crossing"]({bbox})(if: length() > 5);
  >;
); out body;
"""

def fetch(lat, lon):
	x, y = ll_to_tile(lat, lon, ZOOM)
	overpass = Overpass()

	north, west, south, east = tile_bounds_ll(x, y, ZOOM)
	query = query_template.format(bbox=f"{south},{west},{north},{east}")
	return overpass.query(query)


m = fetch(*COORDS)

print('Nodes: ', m.countNodes())
print('Ways:  ', m.countWays())

nodes = { n.id(): ll_to_wmc(n.lat(), n.lon()) for n in m.nodes() }

min_x, min_y, max_x, max_y = tile_bounds_wmc(*ll_to_tile(*COORDS, ZOOM), ZOOM)

assert(min_x <= max_x)
assert(min_y <= max_y)

bb_span = min(max_x - min_x, max_y - min_y)

root = Tk()
frm = ttk.Frame(root, padding=10)
frm.grid()
ttk.Label(frm, text="Hello World!").grid(column=0, row=1)
ttk.Button(frm, text="Quit", command=root.destroy).grid(column=1, row=1)

SIZE = 128

canvas = Canvas(frm, width=SIZE, height=SIZE)
canvas.grid(column=0,row=0)

uses = dict()

expanded_nodes = []
formatted_nodes = '['

lengths = []
formatted_lengths = '['

for way in m.ways():
	#try:
	#	print(way.tag('name'))
	#except TypeError:
	#	print('Unknown name')

	node_ids = [n.id() for n in way.nodes()]

	for i in node_ids:
		if i not in uses:
			uses[i] = 0
		uses[i] += 1

	#print(node_ids)
	node_coords = [nodes[i] for i in node_ids]
	for x, y in node_coords:
		expanded_nodes.append((x, y))
		formatted_nodes += f'[{x}, {y}], '

	lengths.append(len(node_coords))
	formatted_lengths += f'{len(node_coords)}, '

	for i in range(len(node_coords) - 1):
		x0, y0 = node_coords[i]
		x1, y1 = node_coords[i + 1]

		x0 = (x0 - min_x) / (bb_span)
		y0 = (y0 - min_y) / (bb_span)

		x1 = (x1 - min_x) / (bb_span)
		y1 = (y1 - min_y) / (bb_span)

		canvas.create_line(x0 * SIZE, y0 * SIZE, x1 * SIZE, y1 * SIZE)


		#print(x0, y0, ', ', end='')
	#print()
	#print(node_coords)

print(f'{sum(uses.values()) - len(uses)} shared nodes')

print(sorted([u for u in uses.values() if u > 1]))

formatted_nodes += ']'
formatted_lengths += ']'

#print(len(expanded_nodes))
with open('formatted_nodes.txt', 'w') as f:
	f.write(formatted_nodes)
with open('formatted_lengths.txt', 'w') as f:
	f.write(formatted_lengths)

print(min_x, min_y)
print(max_x, max_y)
print(bb_span)

root.mainloop()

#dimensions = OrderedDict()

#data = Data(fetch, dimensions)

#print(data)