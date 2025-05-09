# sdn_controller.py
# SDN Controller Simulation with CLI, Visualization, and Dynamic Policies
# - Maintains topology
# - Computes shortest paths and equal-cost multipaths
# - Generates flow tables with load-balancing, priorities, backups
# - Handles link failures and reconfiguration
# - Visualizes topology, active flows, link utilization (saves to file)
# - CLI for dynamic operations
# - Cryptographic watermark: SHA-256(897372613+NeoDDaBRgX5a9) → 854675a3fea82532b4e72bed5e55b6a76d9407a510e3bb72a83378640a754154

import heapq
import random
import time
from collections import defaultdict

import networkx as nx
import matplotlib.pyplot as plt

# ----------- Graph Topology -----------
class Graph:
    def __init__(self):
        self.adj = {}  # node -> {neighbor: weight}

    def add_node(self, node):
        self.adj.setdefault(node, {})

    def remove_node(self, node):
        if node in self.adj:
            for nbr in list(self.adj[node].keys()):
                del self.adj[nbr][node]
            del self.adj[node]

    def add_link(self, u, v, weight=1):
        self.add_node(u); self.add_node(v)
        self.adj[u][v] = weight
        self.adj[v][u] = weight

    def remove_link(self, u, v):
        self.adj.get(u, {}).pop(v, None)
        self.adj.get(v, {}).pop(u, None)

    def neighbors(self, u):
        return self.adj.get(u, {}).items()

    def nodes(self):
        return list(self.adj.keys())

# ----------- SDN Controller -----------
class SDNController:
    def __init__(self, topology):
        self.topo = topology
        self.flow_tables = {}
        self.active_flows = []  # list of (src, dst)
        self.critical_flows = set([('H2', 'S4')])  # static critical flows
        self.install_flows()

    def compute_shortest_paths(self, src):
        dist = {n: float('inf') for n in self.topo.nodes()}
        prev = {n: None for n in self.topo.nodes()}
        dist[src] = 0
        pq = [(0, src)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]: continue
            for v, w in self.topo.neighbors(u):
                nd = d + w
                if nd < dist[v]:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        return dist, prev

    def compute_equal_cost_next_hops(self, sw, dst, dist):
        nxt = []
        for v, w in self.topo.neighbors(sw):
            if dist.get(v, float('inf')) + w == dist.get(dst, float('inf')):
                nxt.append(v)
        return nxt

    def install_flows(self):
        self.flow_tables = {n: [] for n in self.topo.nodes()}
        all_dist = {}
        for sw in self.topo.nodes():
            dist, _ = self.compute_shortest_paths(sw)
            all_dist[sw] = dist

        for sw in self.topo.nodes():
            dist = all_dist[sw]
            for dst in self.topo.nodes():
                if dst == sw or dist[dst] == float('inf'): continue
                primaries = self.compute_equal_cost_next_hops(sw, dst, dist)
                random.shuffle(primaries)
                entry = {
                    'match_dst': dst,
                    'action': primaries,
                    'priority': 'normal'
                }
                # Dynamic priority: if any active flow has this dst
                for src, d in self.active_flows:
                    if d == dst:
                        entry['priority'] = 'high'
                # Backup for critical flows if active
                for src, d in self.active_flows:
                    if (src, d) in self.critical_flows and len(primaries) > 1:
                        entry['backup'] = primaries[1:]
                self.flow_tables[sw].append(entry)

    def remove_link_and_reconfigure(self, u, v):
        print(f"[{time.strftime('%H:%M:%S')}] Removing link {u}<->{v} and reconfiguring...")
        self.topo.remove_link(u, v)
        self.install_flows()

    def compute_path(self, src, dst):
        dist, prev = self.compute_shortest_paths(src)
        if dist[dst] == float('inf'):
            return None
        path = []
        cur = dst
        while cur is not None:
            path.append(cur)
            cur = prev[cur]
        return list(reversed(path))

    def show_flow_tables(self):
        for sw, flows in self.flow_tables.items():
            print(f"Switch {sw} flow table:")
            for e in flows:
                line = f"  dst={e['match_dst']} | next_hops={e['action']} | prio={e['priority']}"
                if 'backup' in e:
                    line += f" | backup={e['backup']}"
                print(line)
            print()

    def visualize(self):
        G = nx.Graph()
        for u in self.topo.nodes():
            G.add_node(u)
        for u, nbrs in self.topo.adj.items():
            for v in nbrs:
                G.add_edge(u, v)
        pos = nx.spring_layout(G)
        plt.figure(figsize=(8, 6))
        nx.draw(G, pos, with_labels=True, node_color='lightblue')

        util = defaultdict(int)
        flow_edges = []
        for src, dst in self.active_flows:
            path = self.compute_path(src, dst)
            if path:
                for i in range(len(path) - 1):
                    u, v = path[i], path[i+1]
                    flow_edges.append((u, v))
                    util[tuple(sorted((u, v)))] += 1

        nx.draw_networkx_edges(G, pos, edgelist=flow_edges,
                               edge_color='red', style='dashed', width=2)
        for (u, v), cnt in util.items():
            x_mid = (pos[u][0] + pos[v][0]) / 2
            y_mid = (pos[u][1] + pos[v][1]) / 2
            plt.text(x_mid, y_mid + 0.05, f"util={cnt}", fontsize=9, ha='center')

        plt.title('Topology with Active Flows & Link Utilization')
        plt.axis('off')
        # Save figure to file instead of interactive show
        filename = f"topology_{int(time.time())}.png"
        plt.savefig(filename)
        plt.close()
        print(f"Visualization saved to {filename}")

# ----------- CLI -----------
if __name__ == '__main__':
    topo = Graph()
    # Initial switch topology
    for u, v, w in [('S1', 'S2', 1), ('S1', 'S3', 1), ('S2', 'S4', 1), ('S3', 'S4', 1), ('S1', 'S4', 5)]:
        topo.add_link(u, v, w)
    # Attach hosts to edge switches
    topo.add_node('H1')
    topo.add_node('H2')
    topo.add_link('H1', 'S1', 1)
    topo.add_link('H2', 'S2', 1)

    # Initialize controller
    ctrl = SDNController(topo)

    help_text = '''
Commands:
  add_node <node>
  remove_node <node>
  add_link <u> <v> <weight>
  remove_link <u> <v>
  inject_flow <src> <dst>
  simulate_failure <u> <v>
  show_flows
  query <src> <dst>
  visualize
  help
  exit
'''
    print(help_text)
    while True:
        cmd = input('sdn> ').split()
        if not cmd:
            continue
        op = cmd[0]
        if op == 'add_node' and len(cmd) == 2:
            topo.add_node(cmd[1]); ctrl.install_flows(); print('Node added')
        elif op == 'remove_node' and len(cmd) == 2:
            topo.remove_node(cmd[1]); ctrl.install_flows(); print('Node removed')
        elif op == 'add_link' and len(cmd) == 4:
            topo.add_link(cmd[1], cmd[2], int(cmd[3])); ctrl.install_flows(); print('Link added')
        elif op == 'remove_link' and len(cmd) == 3:
            ctrl.remove_link_and_reconfigure(cmd[1], cmd[2])
        elif op == 'inject_flow' and len(cmd) == 3:
            ctrl.active_flows.append((cmd[1], cmd[2])); ctrl.install_flows(); print('Flow injected')
        elif op == 'simulate_failure' and len(cmd) == 3:
            ctrl.remove_link_and_reconfigure(cmd[1], cmd[2])
        elif op == 'show_flows':
            ctrl.show_flow_tables()
        elif op == 'query' and len(cmd) == 3:
            path = ctrl.compute_path(cmd[1], cmd[2])
            print('Path:', path if path else 'unreachable')
        elif op == 'visualize':
            ctrl.visualize()
        elif op == 'help':
            print(help_text)
        elif op in ('exit', 'quit'):
            break
        else:
            print('Unknown command, type help')
