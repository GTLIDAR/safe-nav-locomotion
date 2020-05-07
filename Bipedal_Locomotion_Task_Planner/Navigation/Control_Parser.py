import simplejson as json
import copy
import numpy as np

def dict_compare(d1, d2):
    d1_keys = set(d1.keys())
    d2_keys = set(d2.keys())
    intersect_keys = d1_keys.intersection(d2_keys)
    added = d1_keys - d2_keys
    removed = d2_keys - d1_keys
    modified = {o : (d1[o], d2[o]) for o in intersect_keys if d1[o] != d2[o]}
    same = set(o for o in intersect_keys if d1[o] == d2[o])
    return added, removed, modified, same
    

def parsePermissiveStrategy(filename,outfilename=None):
    automaton = dict()
    file = open(filename)
    varflag = 0
    transflag = 0
    for line in file:
        l = line.split()
        if len(l) > 0:
            if l[0] == 'State':
                transflag = 0
                automaton_state = int(l[1])
                automaton[automaton_state] = dict.fromkeys(['State','Successors'])
                automaton[automaton_state]['State'] = dict()
                automaton[automaton_state]['Successors'] = []
                varflag = 1
            elif varflag == 1:
                for var in l:
                    v = var[0:var.index('@')]
                    if v not in automaton[automaton_state]['State'].keys():
                        automaton[automaton_state]['State'][v] = []
                        automaton[automaton_state]['State'][v].append(int(var[var.index(':')+1]))
                    else:
                        automaton[automaton_state]['State'][v].append(int(var[var.index(':')+1]))
                varflag = 0
                transflag = 1
                for var in automaton[automaton_state]['State'].keys():
                    automaton[automaton_state]['State'][var] = int(''.join(str(e) for e in automaton[automaton_state]['State'][var])[::-1], 2)
            elif transflag == 1:
                automaton[automaton_state]['Successors'].append(int(l[0]))
    if outfilename==None:
        return automaton
    else:
        writeJson(None,outfilename,automaton)


def parseJson(filename,outfilename=None):
    automaton = dict()
    file = open(filename)
    data = json.load(file)
    file.close()
    variables = dict()
    for var in data['variables']:
            v = var.split('@')[0]
            if v not in variables.keys():
                for var2ind in range(data['variables'].index(var),len(data['variables'])):
                    var2 = data['variables'][var2ind]
                    if v != var2.split('@')[0]:
                        variables[v] = [data['variables'].index(var), data['variables'].index(var2)]
                        break
                    if data['variables'].index(var2) == len(data['variables'])-1:
                        variables[v] = [data['variables'].index(var), data['variables'].index(var2)+1]

    for s in data['nodes'].keys():
        automaton[int(s)] = dict.fromkeys(['State','Successors'])
        automaton[int(s)]['State'] = dict()
        automaton[int(s)]['Successors'] = []
        for v in variables.keys():
            if variables[v][0] == variables[v][1]:
                bin  = [data['nodes'][s]['state'][variables[v][0]]]
            else:
                bin = data['nodes'][s]['state'][variables[v][0]:variables[v][1]]
            automaton[int(s)]['State'][v] = int(''.join(str(e) for e in bin)[::-1], 2)
            automaton[int(s)]['Successors'] = data['nodes'][s]['trans']
    if outfilename==None:
        return automaton
    else:
        writeJson(None,outfilename,automaton)

def writeJson(infile,outfile,dict=None):
    if dict is None:
        dict = parseJson(infile)
    j = json.dumps(dict, indent=1)
    f = open(outfile, 'w')
    print >> f, j
    f.close()


def findstate(automaton,statedict):
    states = set()
    for s in automaton:
        added, removed, modified, same = dict_compare(automaton[s]['State'], statedict)
        if len(modified) == 0:
            states.add(s)
    return states

def computeAutomatonState(automaton,currstate,state):
    allautstates = set()
    for autstate in automaton[currstate]['Successors']:
        added, removed, modified, same = dict_compare(automaton[autstate]['State'], state)
        if len(modified) == 0:
            allautstates.add(autstate)
    return allautstates

if __name__ == '__main__':
    parseJson('Examples/scheduler.json','Examples/scheduler_readable.json')
    # Put in the file location of the json file and the desired output file name here
