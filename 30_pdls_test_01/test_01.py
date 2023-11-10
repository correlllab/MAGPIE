#!/usr/bin/env python

########## INIT ####################################################################################

##### Imports #####

### Standard ###
from __future__ import print_function
import os, sys

### Local ###
sys.path.append( "../pddlstream/" )
from pddlstream.algorithms.meta import solve, create_parser
from pddlstream.algorithms.search import solve_from_pddl
from pddlstream.utils import read
from pddlstream.language.constants import print_solution, PDDLProblem



########## PARSING #################################################################################

# def fetch_init( fPath ):
#     """ Get the init info as a list of tuples of strings """

#     initConds = []
    
#     # 1. Open problem spec && Read lines
#     with open( fPath, "r" ) as probFile:
#         probLines = probFile.readlines()
        
#         # 2. For each line in the file
#         for line in probLines: 

#             # 3. If this is the init spec
#             if ":init" in line:
#                 # NO THIS IS SILLY

########## PDDL FUNCTIONS ##########################################################################


def read_pddl( filename ):
    """ Read the PDDL file in this directory """
    # NOTE: This function assumes that `filename` is in the same directory as this PY file
    directory = os.path.dirname( os.path.abspath( __file__ ) )
    return read( os.path.join( directory, filename ) )


# def solve_pddl():
#     domain_pddl  = read_pddl( 'domain.pddl'  )
#     problem_pddl = read_pddl( 'problem.pddl' )

#     plan, cost = solve_from_pddl( domain_pddl, problem_pddl )
#     print( 'Plan:', plan )
#     print( 'Cost:', cost )


def get_problem( domainFile ):
    """ Construct a particular problem to solve """
    # NOTE: Initial conditions and goal are hard-coded by this function
    domain_pddl  = read_pddl( domainFile )
    constant_map = {}
    stream_pddl  = None
    stream_map   = {}
    init = [
        ('on-table', 'r',),
		('on-table', 'g',),
		('on-table', 'b',),
		('clear', 'r',),
		('clear', 'g',),
		('clear', 'b',),
		('arm-empty',),
    ]
    goal = ('and',
        ('on', 'g', 'b',),
        ('on', 'r', 'g',),
    )
    return PDDLProblem( domain_pddl, constant_map, stream_pddl, stream_map, init, goal )

def solve_pddlstream( domainFile, debug = False ):
    """ Solve the problem specified by the `domainFile` and `get_problem` """
    parser = create_parser()
    args = parser.parse_args()
    print( 'Arguments:', args )
    problem = get_problem( domainFile )
    planner = 'lmcut-astar' # cerberus
    solution = solve(
        problem, 
        algorithm  = args.algorithm, 
        unit_costs = args.unit, 
        planner    = planner, 
        debug      = debug
    )
    print_solution( solution )



########## MAIN ####################################################################################

if __name__ == "__main__":
    solve_pddlstream( "blocks_domain.pddl" )