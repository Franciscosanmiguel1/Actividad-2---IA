#!/usr/bin/env python3
"""
plan_de_rutas.py
Uso: python plan_de_rutas.py rules.txt ORIGEN DESTINO
Ejemplo: python plan_de_rutas.py rules.txt A E
"""

import sys
import re
import ast
import heapq
from collections import defaultdict, namedtuple

Edge = namedtuple('Edge', ['to','time','line'])

def parse_rules(path):
    stations = set()
    closed = set()
    graph = defaultdict(list)
    transfer_penalty = 0

    func_re = re.compile(r'^\s*([a-zA-Z_][a-zA-Z0-9_]*)\s*\((.*)\)\s*(?:#.*)?$')

    with open(path, 'r', encoding='utf-8') as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith('#'):
                continue
            m = func_re.match(line)
            if not m:
                print(f"Warning: línea no entendida -> {line}")
                continue
            func, args_text = m.groups()
            tuple_text = '(' + args_text + ')'
            try:
                args = ast.literal_eval(tuple_text)
            except Exception as e:
                print(f"Error parseando argumentos en: {line}\n  {e}")
                continue

            if not isinstance(args, tuple):
                args = (args,)

            func = func.lower()
            if func == 'station':
                stations.add(str(args[0]))
            elif func == 'closed':
                closed.add(str(args[0]))
            elif func == 'edge':
                if len(args) < 4:
                    print(f"edge requiere 4 argumentos: {line}")
                    continue
                a, b, t, lineid = args[0], args[1], args[2], args[3]
                graph[str(a)].append(Edge(str(b), float(t), str(lineid)))
            elif func == 'transfer_penalty':
                transfer_penalty = float(args[0])
            else:
                print(f"Función desconocida: {func} (línea: {line})")

    return stations, closed, graph, transfer_penalty

def dijkstra_with_lines(start, goal, graph, closed, transfer_penalty):
    heap = []
    heapq.heappush(heap, (0.0, start, None, [(start, None)]))
    visited = dict()

    while heap:
        time_so_far, node, cur_line, path = heapq.heappop(heap)
        if (node,cur_line) in visited and visited[(node,cur_line)] <= time_so_far:
            continue
        visited[(node,cur_line)] = time_so_far

        if node == goal:
            return time_so_far, path

        if node in closed:
            continue

        for e in graph.get(node, []):
            if e.to in closed:
                continue
            extra = 0.0
            if cur_line is not None and e.line != cur_line:
                extra = transfer_penalty
            new_time = time_so_far + e.time + extra
            new_path = path + [(e.to, e.line)]
            if ((e.to, e.line) not in visited) or (visited.get((e.to,e.line), float('inf')) > new_time):
                heapq.heappush(heap, (new_time, e.to, e.line, new_path))

    return None, None

def pretty_print_route(total_time, path):
    if path is None:
        print("No se encontró ruta válida.")
        return
    print(f"\nRuta encontrada: tiempo total estimado = {total_time:.2f} minutos")
    for i, (node, line) in enumerate(path):
        if i == 0:
            print(f"  {node} (inicio)")
        else:
            print(f"  -> {node} via línea {line}")

def main():
    if len(sys.argv) < 4:
        print("Uso: python route_planner.py rules.txt ORIGEN DESTINO")
        sys.exit(1)
    rules = sys.argv[1]
    origen = sys.argv[2]
    destino = sys.argv[3]

    stations, closed, graph, transfer_penalty = parse_rules(rules)
    print(f"Estaciones cargadas: {len(stations)}  Cerradas: {len(closed)}  Transfer penalty: {transfer_penalty}")

    time, path = dijkstra_with_lines(origen, destino, graph, closed, transfer_penalty)
    pretty_print_route(time, path)

if __name__ == "__main__":
    main()
