#!/usr/bin/env python

########## INIT ####################################################################################

from __future__ import print_function

import os, sys

sys.path.append( "../pddlstream/" )

from pddlstream.algorithms.meta import solve, create_parser
from pddlstream.algorithms.search import solve_from_pddl
from pddlstream.utils import read
from pddlstream.language.constants import print_solution, PDDLProblem



########## PDDL FUNCTIONS ##########################################################################

def read_pddl( filename ):
    directory = os.path.dirname( os.path.abspath( __file__ ) )
    return read( os.path.join( directory, filename ) )

def solve_pddl():
    domain_pddl  = read_pddl( 'domain.pddl'  )
    problem_pddl = read_pddl( 'problem.pddl' )

    plan, cost = solve_from_pddl( domain_pddl, problem_pddl )
    print( 'Plan:', plan )
    print( 'Cost:', cost )

# FIXME: START HERE