from OSMPythonTools.overpass import Overpass, overpassQueryBuilder
from OSMPythonTools.data import Data
from collections import OrderedDict
import pickle
import math

from tkinter import *
from tkinter import ttk

def fetch():
	# query = overpassQueryBuilder(bbox=(40.44020,-79.94743,40.44706,-79.93284), elementType='way', out='body')
	query = '(way[highway](40.4402,-79.94743,40.44706,-79.93284); >;); out body;'
	print(query)


	try:
		with open('cached.pickle', 'rb') as f:
			m = pickle.load(f)
	except FileNotFoundError:
		overpass = Overpass()

		_ = '''
		(
			node(40.44020,-79.94743,40.44706,-79.93284);
			<;
		);
		out meta;
		'''

		m = overpass.query(query)

		with open('cached.pickle', 'wb') as f:
			pickle.dump(m, f)
	
	return m

m = fetch()

print('Nodes: ', m.countNodes())
print('Ways:  ', m.countWays())

def ll_to_wmc(lat, lon):
	lat = math.radians(lat)
	lon = math.radians(lon)

	x = (1 / (2 * math.pi)) * (math.pi + lon)
	y = (1 / (2 * math.pi)) * (math.pi - math.log(math.tan((math.pi / 4) + lat / 2.0)))
	return x, y

nodes = { n.id(): ll_to_wmc(n.lat(), n.lon()) for n in m.nodes() }

min_x, min_y = ll_to_wmc(40.44706,-79.94743)
max_x, max_y = ll_to_wmc(40.4402,-79.93284)

assert(min_x <= max_x)
assert(min_y <= max_y)

bb_span = min(max_x - min_x, max_y - min_y)

root = Tk()
frm = ttk.Frame(root, padding=10)
frm.grid()
ttk.Label(frm, text="Hello World!").grid(column=0, row=1)
ttk.Button(frm, text="Quit", command=root.destroy).grid(column=1, row=1)

SIZE = 64

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