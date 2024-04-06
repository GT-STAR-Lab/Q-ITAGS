import copy
import json
import math
import random

import networkx as nx

random.seed(111)

input_graph_name = "polypixel"
input_graph = nx.read_graphml(f'{input_graph_name}.graphml')

output_graph = {
    "configuration_type": "graph",
    "graph_type": "complete_sampled_point",
    "vertices": [],
    "edges": []
}

num_nodes = len(input_graph.nodes())
for vertex in input_graph.nodes():
    i = int(input_graph.nodes[vertex]["name"])
    x = input_graph.nodes[vertex]["x"]
    y = input_graph.nodes[vertex]["y"]
    output_graph["vertices"].append({
        "id": i,
        "x": x,
        "y": y
    })


def distance(a, b):
    return math.sqrt((a["x"] - b["x"]) ** 2 + (a["y"] - b["y"]) ** 2)


for edge in input_graph.edges():
    cost = distance(input_graph.nodes[edge[0]], input_graph.nodes[edge[1]])
    input_graph.edges[edge]['cost'] = cost
    output_graph["edges"].append({
        "vertex_a": edge[0],
        "vertex_b": [1],
        "cost": cost
    })

with open(f'{input_graph_name}.json', 'w') as f:
    json.dump(output_graph, f, sort_keys=True, indent=4)
output_graph["edges"] = []

paths = dict(nx.shortest_path_length(input_graph, weight='cost'))
for j in range(num_nodes):
    for k in range(j + 1, num_nodes):
        output_graph["edges"].append({
            "vertex_a": j,
            "vertex_b": k,
            "cost": paths[f'n{j}'][f'n{k}']
        })
with open(f'{input_graph_name}_complete.json', 'w') as f:
    json.dump(output_graph, f, sort_keys=True, indent=4)
output_graph["edges"] = []

p = 0.1
for i in range(100):
    graph_copy = copy.deepcopy(input_graph)
    graph_copy.remove_edges_from(random.sample(graph_copy.edges(), k=int(p * graph_copy.number_of_edges())))
    paths = dict(nx.shortest_path_length(graph_copy, weight='cost'))

    output_graph["edges"].append([])
    for j in range(num_nodes):
        if f'n{j}' in paths:
            for k in range(j + 1, num_nodes):
                if f'n{k}' in paths[f'n{j}']:
                    output_graph["edges"][-1].append({
                        "vertex_a": j,
                        "vertex_b": k,
                        "cost": paths[f'n{j}'][f'n{k}']
                    })
    if (i + 1) % 10 == 0:
        with open(f'{input_graph_name}_sampled_{i + 1}.json', 'w') as f:
            json.dump(output_graph, f, sort_keys=True, indent=4)