#!/usr/bin/python
#
# Translates from LittleMOP to the Slugs format

import math
import os
import sys
import resource
import subprocess
import signal
from parser import Parser
from re import match
import StringIO


# Allocate global parser
p = Parser()

# =====================================================
# Lexer for the PSL formulas
# =====================================================
def tokenize(str):

    res = []
    while str:
        # Ignoring stuff        
        if str[0].isspace() or (str[0]=='\n'):
            str = str[1:]
            continue

        if str.startswith("FALSE"):
            str = str[5:]
            res.append(("FALSE",))
            continue

        if str.startswith("TRUE"):
            str = str[4:]
            res.append(("TRUE",))
            continue

        if str.startswith("next"):
            str = str[4:]
            res.append(("next",))
            continue

        m = match('[a-zA-Z0-9_.]+', str)
        if m:
            # Special case: ["X","F","U","W","0","1"]
            if m.end()==1:
                if str[0] in ["X","F","G","U","W","0","1"]:
                    res.append((str[0],))
                    str = str[1:]
                else:
                    res.append(('id', m.group(0)))
                    str = str[m.end(0):]
            else:
                res.append(('id', m.group(0)))
                str = str[m.end(0):]
            continue

        res.append((str[0],))
        str = str[1:]
    return res

# =====================================================
# Simplify the specifications
# =====================================================
def clean_tree(tree):
    """ Cleans a parse Tree, i.e. removes brackets and so on """
    if tree[0] in p.terminals:
        return tree
    if (tree[0]=="Brackets"):
        return clean_tree(tree[2])
    elif (tree[0]=="Implication") and (len(tree)==2):
        return clean_tree(tree[1])
    elif (tree[0]=="Atomic") and (len(tree)==2):
        return clean_tree(tree[1])
    elif (tree[0]=="Conjunction") and (len(tree)==2):
        return clean_tree(tree[1])
    elif (tree[0]=="Biimplication") and (len(tree)==2):
        return clean_tree(tree[1])
    elif (tree[0]=="Disjunction") and (len(tree)==2):
        return clean_tree(tree[1])
    elif (tree[0]=="Xor") and (len(tree)==2):
        return clean_tree(tree[1])
    elif (tree[0]=="BinaryTemporalFormula") and (len(tree)==2):
        return clean_tree(tree[1])
    elif (tree[0]=="UnaryFormula") and (len(tree)==2):
        return clean_tree(tree[1])
    elif (tree[0]=="AtomicFormula"):
        if len(tree)!=2:
            raise ValueError("AtomicFormula must have length 2")
        return clean_tree(tree[1])
    elif (tree[0]=="Implication"):
        return [tree[0],clean_tree(tree[1]),clean_tree(tree[3])]
    elif (tree[0]=="Conjunction"):
        return [tree[0],clean_tree(tree[1]),clean_tree(tree[3])]
    elif (tree[0]=="Biimplication"):
        return [tree[0],clean_tree(tree[1]),clean_tree(tree[3])]
    elif (tree[0]=="Disjunction"):
        return [tree[0],clean_tree(tree[1]),clean_tree(tree[3])]
    elif (tree[0]=="Xor"):
        return [tree[0],clean_tree(tree[1]),clean_tree(tree[3])]
    elif (tree[0]=="BinaryTemporalFormula"):
        return [tree[0],clean_tree(tree[1]),clean_tree(tree[2]),clean_tree(tree[3])]
    elif (tree[0]=="UnaryFormula"):
        return [tree[0],clean_tree(tree[1]),clean_tree(tree[2])]
    elif (tree[0]=="BinaryTemporalOperator"):
        # Remove the "superfluous indirection"
        return clean_tree(tree[1])
    elif (tree[0]=="UnaryTemporalOperator"):
        # Remove the "superfluous indirection"
        return clean_tree(tree[1])
    elif (tree[0]=="Assignment"):
        # Flatten "id" case
        A = [tree[0],tree[1][1]]
        A.extend(tree[2:])
        return A
    else:
        A = [tree[0]]
        for x in tree[1:]:
           A.append(clean_tree(x))
        return A

def flatten_as_much_as_possible(tree):
    """ Flattens nested disjunctions/conjunctions """
    # Ground case?
    if len(tree)==1:
        return tree
    if (type(tree)==type("A")) or (type(tree)==type(u"A")): # TODO: How to do this in the way intended?
        return tree
    newTree = []
    for a in tree:
        newTree.append(flatten_as_much_as_possible(a))
    tree = newTree

    # Conjunction
    if (tree[0]=="Conjunction"):
        parts = [tree[0]]
        for a in tree[1:]:
            if a[0]=="Conjunction":
                parts.extend(a[1:])
            else:
                parts.append(a)
        return parts

    # Disjunction
    if (tree[0]=="Disjunction"):
        parts = [tree[0]]
        for a in tree[1:]:
            if a[0]=="Disjunction":
                parts.extend(a[1:])
            else:
                parts.append(a)
        return parts

    # Xor
    if (tree[0]=="Xor"):
        parts = [tree[0]]
        for a in tree[1:]:
            if a[0]=="Xor":
                parts.extend(a[1:])
            else:
                parts.append(a)
        return parts

    # Every other case
    return tree



# =====================================================
# The Parsing function
# =====================================================
def parseLTL(ltlTxt):

    try:
        input = tokenize(ltlTxt)
        tree = p.parse(input)

    except p.ParseErrors, exception:
        for t,e in exception.errors:
            if t[0] == p.EOF:
                print >>sys.stderr, "Formula end not expected here"
                continue

            found = repr(t[0])
            if len(e) == 1:
                print >>sys.stderr, "Error in LTL formula: "+ltlTxt
                print >>sys.stderr, "Expected %s, but found %s " %(repr(e[0]), found)
            else:
                print >>sys.stderr, "Error in LTL formula: "+ltlTxt
                print >>sys.stderr, "Could not parse %s, "%found
                print >>sys.stderr, "Wanted a token of one of the following forms: "+", ".join([ repr(s) for s in e ])
        raise

    # Convert to a tree
    cleaned_tree = flatten_as_much_as_possible(clean_tree(tree))
    return cleaned_tree

def performConversion(smvFile, ltlFile):
    # Read LTL file
    ltlFileStream = open(ltlFile,"r")
    mode = 0
    assumptions = ""
    guarantees = ""
    for line in ltlFileStream.readlines():
        line = line.strip()
        if (line=="LTLSPEC -- Assumptions"):
            mode = 1
        elif (line=="LTLSPEC -- Guarantees"):
            mode = 2
        else:
            if mode == 1:
                assumptions = assumptions + " " + line
            elif mode == 2:
                guarantees = guarantees + " " + line
    ltlFileStream.close()

    assumptionTree = parseLTL(assumptions)
    guaranteeTree = parseLTL(guarantees)

    # Read SMV file
    smvFileStream = open(smvFile,"r")
    mode = 0
    inputBits = []
    outputBits = []
    for line in smvFileStream.readlines():
        line = line.strip()
        if (line.startswith("MODULE env")):
            mode = 1
        elif (line.startswith("MODULE sys")):
            mode = 2
        else:
            if line.find(" : ")>0:
                if mode == 1:
                    inputBits.append(line.split(" : ")[0])
                elif mode == 2:
                    outputBits.append(line.split(" : ")[0])
    smvFileStream.close()

    # ==================================
    # Build Slugs file - I/O Variables
    # ==================================
    print "[INPUT]"
    for bit in inputBits:
        print bit
    print ""
    print "[OUTPUT]"
    for bit in outputBits:
        print bit
    print ""

    # Iterate over the property types.
    buildPropertySet(assumptionTree,"ENV_",False,False)
    buildPropertySet(assumptionTree,"ENV_",False,True)
    buildPropertySet(guaranteeTree,"SYS_",True,False)
    buildPropertySet(guaranteeTree,"SYS_",True,True)


# ============================================
# Build Slugs file - Temporal logic properties
# ============================================
def parseSimpleFormula(tree, isPrimed):
    if (tree[0]=="Biimplication"):
        b1 = parseSimpleFormula(tree[1],isPrimed)
        b2 = parseSimpleFormula(tree[2],isPrimed)
        return ["|","&","!"]+b1+["!"]+b2+["&"]+b1+b2
    if (tree[0]=="Implication"):
        b1 = parseSimpleFormula(tree[1],isPrimed)
        b2 = parseSimpleFormula(tree[2],isPrimed)
        return ["|","!"]+b1+b2
    if (tree[0]=="Conjunction"):
        ret = parseSimpleFormula(tree[1],isPrimed)
        for a in tree[2:]:
            ret = ["&"]+ret+parseSimpleFormula(a,isPrimed)
        return ret
    if (tree[0]=="Disjunction"):
        ret = parseSimpleFormula(tree[1],isPrimed)
        for a in tree[2:]:
            ret = ["|"]+ret+parseSimpleFormula(a,isPrimed)
        return ret
    if (tree[0]=="UnaryFormula"):
        if tree[1][0]=="NotOperator":
            return ["!"]+parseSimpleFormula(tree[2],isPrimed)
        elif tree[1][0]=="NextOperator":
            if isPrimed:
                raise "Nested nexts are not allowed."
            return parseSimpleFormula(tree[2],True)
    if (tree[0]=="Assignment"):
        var = tree[1]
        if var[1]!=".":
            raise "No LTLMop input: The dot must be the second character in every atomic proposition in the LTL file"
        var = var[2:]
        if isPrimed:
            var = var + "'"
        return [var]
    if (tree[0]=="TRUE"):
        return ["1"]
    if (tree[0]=="FALSE"):
        return ["0"]
    else:
        print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        print tree
        raise 123
    

def buildPropertySet(tree,fileprefix,startWithNextOperatorForLivenessProperties,handleInitialisationProperties):
    if (tree[0]!="Formula"):
        raise "Wrong tree type"
    tree = tree[1]    
    conjuncts = []
    if tree[0]=="Conjunction":
        conjuncts = tree[1:]
    else:
        conjuncts = [tree]

    # Iterate over the types
    initializationProperties = []
    safetyProperties = []
    livenessProperties = []
    
    for c in conjuncts:
        if c[0]=="UnaryFormula":
            if c[1][0]!="GloballyOperator":
                initializationProperties.append(parseSimpleFormula(c,False))
            else:
                rest = c[2]
                if rest[0]=="UnaryFormula":
                    if rest[1][0]!="FinallyOperator":
                        safetyProperties.append(parseSimpleFormula(rest,False))
                    else:
                        livenessProperties.append(parseSimpleFormula(rest[2],startWithNextOperatorForLivenessProperties))
                else:
                    safetyProperties.append(parseSimpleFormula(rest,False))
        else:
            initializationProperties.append(parseSimpleFormula(c,False))

    if handleInitialisationProperties:
        print "["+fileprefix+"INIT]"
        for a in initializationProperties:
            print " ".join(a)
        print ""
    else:
        print "["+fileprefix+"TRANS]"
        for a in safetyProperties:
            print " ".join(a)
        print ""

        print "["+fileprefix+"LIVENESS]"
        for a in livenessProperties:
            print " ".join(a)
        print ""

# ==================================
# Entry point
# ==================================
if __name__ == "__main__":
    if len(sys.argv)<3:
        print >>sys.stderr, "Error: Need SMV and LTL files as parameter"
        sys.exit(1)

    smvFile = sys.argv[1]
    ltlFile = sys.argv[2]
    performConversion(smvFile, ltlFile)


