import re
import networkx as nx
from networkx.classes.digraph import DiGraph

from parser import parse as parse_guard



def find_symbols(formula):
    regex = re.compile(r"[a-z]+[a-z0-9]*")
    matches = regex.findall(formula)
    symbols = list()
    for match in matches:
        symbols += [match]
    symbols = list(set(symbols))
    symbols.sort()
    return symbols


def find_states(edges, dot_text):
    initial = set()
    accept = set()
    for (f,t) in edges.keys():
        if f == 'init':
            initial.add(t)

    research = re.search(r'doublecircle];\s(\d+)', dot_text)
    accept.add(research.group(1))
    
    return (list(initial), list(accept))

def getDFA(path, formula):
    fp= open(path)
    test_gv = fp.read()
    fp.close()

    G = DiGraph(nx.nx_pydot.read_dot(path))

    initial, accept = find_states(G.edges, test_gv)
    symbols = find_symbols(formula)

    DFA = DiGraph(initial=initial, accept=accept, symbols=symbols)

    # G.graph['initial'] = initial
    # G.graph['accept'] = accept
    for nd in G.nodes:
        if nd == 'init':
            continue
        DFA.add_node(nd)

    for (ef,et) in G.edges.keys():
        if ef == 'init':
            continue
        guard_formula = G.edges[(ef,et)]['label'].strip('"')
        guard_formula = guard_formula.replace('~', '!')
        guard_formula = guard_formula.replace('&', '&&')
        guard_formula = guard_formula.replace('|', '||')
        guard_formula = guard_formula.replace('true', '1')

        guard_expr = parse_guard(guard_formula)
        # G[ef][et]["guard"] = guard_expr
        DFA.add_edge(ef, et, guard=guard_expr, guard_formula=guard_formula)


    return DFA

import os
import subprocess
root_dir = r'/home/dyl/yzchen_ws/task_ws/hmi_misc/'
ltl_path = os.path.join(root_dir, 'output_ltl.txt')
buchi_path = os.path.join(root_dir, 'rqt_dfa.gv')

def call_LTLf2DFA(formula):

    with open(ltl_path, 'w') as fp:
        fp.write(formula)
    # translate to DFA
    ltl2aut_cmd =  'python3 /home/dyl/yzchen_ws/task_ws/LTLf2DFA/ltlf2dfa.py'
    process = subprocess.Popen(ltl2aut_cmd.split(), stdout=subprocess.PIPE)                
    output, error = process.communicate()

    # directly read gv
    DFA = getDFA(buchi_path, formula)

    return DFA

if __name__ == '__main__':
    path = "mona.gv"
    formula = '<> (pond && <> grassland) && <>[] pond'
    dfa = getDFA(path, formula)

    # regex = re.match(r"doublecircle];\s([0-9])", test_gv)

    # research = re.search(r'doublecircle];\s(\d+)', test_gv)





    print('TODO: get accept')
