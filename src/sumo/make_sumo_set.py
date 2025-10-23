import xml.etree.ElementTree as ET
import os, random, subprocess

# ============= Control Variables ===============
OUTAGE_ZONE = 800
AV_SPEED = 40           # 평균 속도 (km/h). 0이면 에피소드마다 임의로 설정
DENSITY = 20            # 평균 밀도 (/1km-lane). 0이면 에피소드마다 임의로 설정
P_GEN = 0.005
NUM_BLOCKS = 6
MAX_STEPS = 3600.0
CORNER_SPEED_LIMIT = 50 / 3.6

# ============= Environment Variables ===============
RSU_RANGE = 800.0                           # Communication range of RSU
EDGE_LENGTH = RSU_RANGE * 2 + OUTAGE_ZONE   # Distance between RSUs
GRID_SIZE = NUM_BLOCKS * EDGE_LENGTH        # Network Size
NUM_LANES = 2                               # Lane Number
SPEED = AV_SPEED / 3.6                      # average speed of vehicles
DEL_SPEED = 0.2                             # delta speed variance
MAX_SPEED = (120 / 3.6) * (1 + DEL_SPEED)   # maximum speed of vehicles
step = GRID_SIZE / NUM_BLOCKS
BASE_PATH=os.path.dirname(__file__)

T_to_INIT = 0.0
L_tot = 0.0
L_path_avg = 0.0

def CalcP_GEN(density):
    n = NUM_BLOCKS - 1
    global L_tot, L_path_avg, T_to_INIT
    L_tot = 2 * (n) * NUM_BLOCKS * EDGE_LENGTH * (2 * NUM_LANES)
    L_path_avg = EDGE_LENGTH * (1 + (2 * (n * n - 1)) / (3 * n))
    v = AV_SPEED if AV_SPEED > 0 else 40.0
    dens = density if density > 0 else 10.0
    T_to_INIT = 3.6 * (L_path_avg / v)
    return (dens * L_tot * v) / (L_path_avg * n * n * 3600.0)

def make_sumo_files():
    global NUM_BLOCKS
    NUM_BLOCKS += 1
    def make_dead_end_nodes(netfile,dead_end_nodes):
        tree = ET.parse(netfile)
        root = tree.getroot()
        for node in root.findall("junction"):
            node_id = node.get("id")
            if node_id in dead_end_nodes: node.set("type", "dead_end")
        tree.write(netfile)

    def _make_axis_positions():
        xs = [0.0] * (NUM_BLOCKS + 1)
        ys = [0.0] * (NUM_BLOCKS + 1)
        for i in range(1, NUM_BLOCKS + 1):
            inc = step / 2.0 if (i == 1 or i == NUM_BLOCKS) else step
            xs[i] = xs[i-1] + inc
            ys[i] = ys[i-1] + inc
        return xs, ys

    def generate_nodes_edges():
        nodes, edges, temp_N = [], [], {}
        xs, ys = _make_axis_positions()
        ndx = 1
        for i in range(NUM_BLOCKS + 1):
            for j in range(NUM_BLOCKS + 1):
                if (i == 0 and (j == 0 or j == NUM_BLOCKS)) or (i == NUM_BLOCKS and (j == 0 or j == NUM_BLOCKS)):
                    continue
                x = xs[i]
                y = ys[j]
                types_ = "dead_end" if i == 0 or j == 0 or i == NUM_BLOCKS or j == NUM_BLOCKS else "traffic_light"
                nodes.append(f'<node id="N{ndx}" x="{x}" y="{y}" type="{types_}"/>')
                temp_N[(i, j)] = f'N{ndx}'
                ndx += 1

        edge_id = 0
        chk_edge = dict()
        for i in range(1, NUM_BLOCKS):
            for j in range(1, NUM_BLOCKS):
                from_node = temp_N[(i, j)]
                for dx, dy in [[0, 1], [1, 0], [0, -1], [-1, 0]]:
                    to_node = temp_N[(i + dx, j + dy)]
                    if SPEED == 0:
                        speed1 = random.uniform(10.0 / 3.6, 120.0 / 3.6)
                        speed2 = random.uniform(10.0 / 3.6, 120.0 / 3.6)
                    else:
                        speed1 = random.uniform(SPEED * (1 - DEL_SPEED), SPEED * (1 + DEL_SPEED))
                        speed2 = random.uniform(SPEED * (1 - DEL_SPEED), SPEED * (1 + DEL_SPEED))
                    ed1 = f'<edge id="E{edge_id}" from="{from_node}" to="{to_node}" numLanes="{NUM_LANES}" speed="{speed1}"/>'
                    ed2 = f'<edge id="-E{edge_id}" from="{to_node}" to="{from_node}" numLanes="{NUM_LANES}" speed="{speed2}"/>'
                    if (from_node, to_node) not in chk_edge:
                        edges.append(ed1)
                        chk_edge[(from_node, to_node)] = edge_id
                    if (to_node, from_node) not in chk_edge:
                        edges.append(ed2)
                        chk_edge[(to_node, from_node)] = edge_id * -1
                    edge_id += 1
        return nodes, edges

    nodes, edges = generate_nodes_edges()
    nodefile = os.path.join(BASE_PATH,"generated.nod.xml")
    edgefile = os.path.join(BASE_PATH,"generated.edg.xml")
    gen_netfile = os.path.join(BASE_PATH,"generated.net.xml")

    with open(nodefile, "w") as f: f.write("<nodes>\n" + "\n".join(nodes) + "\n</nodes>")
    with open(edgefile, "w") as f: f.write("<edges>\n" + "\n".join(edges) + "\n</edges>")
    subprocess.call(['netconvert', '-n', nodefile, '-e', edgefile, '-o', gen_netfile,'--no-turnarounds', '--junctions.limit-turn-speed', str(CORNER_SPEED_LIMIT)])

    tree = ET.parse(nodefile)
    root = tree.getroot()
    border_nodes = {}
    for node in root.findall("node"):
        nid = node.get("id")
        x = float(node.get("x"))
        y = float(node.get("y"))
        if node.get("type") == 'dead_end': border_nodes[nid] = (x, y)
    tree_e = ET.parse(edgefile)
    root_e = tree_e.getroot()
    taz_sources, taz_sinks, taz_point, NT = [], [], {}, dict()
    for edge in root_e.findall("edge"):
        eid = edge.get("id")
        from_node = edge.get("from")
        to_node = edge.get("to")
        if from_node in border_nodes: taz_sources.append(eid); taz_point[eid] = from_node
        if to_node in border_nodes: taz_sinks.append(eid); taz_point[eid] = to_node
    with open(os.path.join(BASE_PATH,"generated.add.xml"), "w") as f:
        f.write('<tazs>\n')
        for idx,i in enumerate(sorted([int(e.split('E')[1]) for e in taz_sinks])):
            x,y = border_nodes[taz_point[f'E{i}']]
            NT[idx]=(x,y)
            shp= " ".join([f'{x+dx},{y+dy}' for dx,dy in [[0,-10],[-10,0],[0,10],[10,0],[0,-10]]])
            f.write(f'  <taz id="taz_{idx}" shape="{shp}" color="blue">\n')
            f.write(f'    <tazSource id="E{i}" weight="1.0"/>\n')
            f.write(f'    <tazSink id="E{i}" weight="1.0"/>\n')
            f.write(f'    <tazSource id="-E{i}" weight="1.0"/>\n')
            f.write(f'    <tazSink id="-E{i}" weight="1.0"/>\n')
            f.write('  </taz>\n')
        f.write('</tazs>\n')
    Taz_len = len(taz_sinks)
    make_dead_end_nodes(gen_netfile, border_nodes)
    with open(os.path.join(BASE_PATH,"generated.rou.xml"), "w") as f:
        f.write('<routes>\n')
        CalcP_GEN(DENSITY)
        global P_GEN
        for i in range(Taz_len):
            for j in range(Taz_len):
                if i == j: continue
                P_GEN = CalcP_GEN(DENSITY) if DENSITY > 0 else CalcP_GEN(random.randint(1, 20))
                f.write(f'  <flow id="F{i}_{j}" begin="0.00" departLane="random" fromTaz="taz_{i}" toTaz="taz_{j}" end="{MAX_STEPS}" probability="{P_GEN / (Taz_len - 1)}"/>\n')
        f.write('</routes>\n')