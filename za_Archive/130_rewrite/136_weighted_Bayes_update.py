########## DEV PLAN ################################################################################
"""
NOTE: Entropy is about the relative height of distribution categories of the readin, not the sum of the categories
"""



########## INIT ####################################################################################

from pprint import pprint

import numpy as np

from utils import get_confusion_matx, get_confused_class_reading
from env_config import _BLOCK_NAMES, _N_CLASSES, _NULL_NAME



########## HELPER FUNCTIONS ########################################################################

def normalize_vec( vec ):
    """ Return a normalized version of this vector if it has magnitude, Otherwise return the zero vector """
    mag = sum( vec )
    if mag > 0.0:
        return vec / mag
    else:
        return vec


def multiclass_Bayesian_belief_update( cnfMtx, priorB, evidnc ):
    """ Update the prior belief using probabilistic evidence given the weight of that evidence """
    Nclass = cnfMtx.shape[0]
    priorB = np.array( priorB ).reshape( (Nclass,1,) )
    evidnc = np.array( evidnc ).reshape( (Nclass,1,) )
    P_e    = cnfMtx.dot( priorB ).reshape( (Nclass,) )
    P_hGe  = np.zeros( (Nclass,Nclass,) )
    for i in range( Nclass ):
        P_hGe[i,:] = (cnfMtx[i,:]*priorB[i,0]).reshape( (Nclass,) ) / P_e
    return P_hGe.dot( evidnc ).reshape( (Nclass,) )


def multiclass_Bayesian_belief_update_modified( cnfMtx, priorB, evidnc ):
    """ Update the prior belief using probabilistic evidence given the weight of that evidence """
    # print( "Evidence:", evidnc, sum( evidnc ) )
    # print( "Normed Evidence:", nrmEvi, sum( nrmEvi ) )
    factor = min( sum( evidnc ), 1.0 )
    nrmEvi = normalize_vec( evidnc )
    nuBelf = multiclass_Bayesian_belief_update( cnfMtx, priorB, nrmEvi )
    belDif = np.subtract( nuBelf, priorB )
    rtnBel = np.add( priorB, belDif*factor )
    return normalize_vec( rtnBel )

    
def extract_dct_values_in_order( dct, keyLst ):
    """ Get the `dct` values in the order specified in `keyLst` """
    rtnLst = []
    for k in keyLst:
        if k in dct:
            rtnLst.append( dct[k] )
    return rtnLst


def extract_class_dist_in_order( obj, order = _BLOCK_NAMES ):
    """ Get the discrete class distribution, in order according to environment variable """
    if isinstance( obj, dict ):
        return np.array( extract_dct_values_in_order( obj, order ) )
    else:
        return np.array( extract_dct_values_in_order( obj.labels, order ) )


def get_uniform_prior_over_labels( orderedLabels = _BLOCK_NAMES ):
    """ Return a discrete distribution with uniform confusion between classes other than `label` """
    rtnLabels = {}
    Nclass    = len( orderedLabels )
    perProb   = 1.0 / Nclass
    for i in range( Nclass ):
        blkName_i = orderedLabels[i]
        rtnLabels[ blkName_i ] = perProb
    return rtnLabels


def integrate_one_reading( priorDct, confMatx, readingDct, order = _BLOCK_NAMES ):
    """ Simulate one noisy reading """
    priorB = extract_class_dist_in_order( priorDct  , order )
    evidnc = extract_class_dist_in_order( readingDct, order )
    rtnBel = multiclass_Bayesian_belief_update_modified( confMatx, priorB, evidnc )
    rtnDct = {}
    for i, label in enumerate( order ):
        rtnDct[ label ] = rtnBel[i]
    return rtnDct


def dct_total( dct ):
    """ Sum values stored in the `dstDct` """
    tot = 0.0
    for v in dct.values():
        tot += v
    return tot


def normalize_dct( dct ):
    """ Return a normalized version of this dictionary if its keys have magnitude, Otherwise return the zero vector """
    mag = dct_total( dct )
    rtn = {}
    if mag > 0.0:
        for k, v in dct.items():
            rtn[k] = 1.0 * v / mag
        return rtn
    else:
        return dct.copy()


########## MAIN ####################################################################################
confProb = 0.05
bFactor  = 0.50
largProb = confProb * 1.75
confMatx = get_confusion_matx( _N_CLASSES, confProb )

grnRead = get_confused_class_reading( 'grnBlock', confProb, _BLOCK_NAMES )
lrgRead = get_confused_class_reading( 'grnBlock', largProb, _BLOCK_NAMES )
sclRead = {}
for label in grnRead.keys():
    sclRead[ label ] = grnRead[ label ] * bFactor

print( "##### Example Readings #####\n" )
print( "## Confused Reading ##" )
pprint( lrgRead )
print( f"Dist. sums to 1.0? {dct_total( lrgRead )}\n" )

print( "## Scaled Reading ##" )
pprint( sclRead )
print( f"Dist. sums to 1.0? {dct_total( sclRead )}\n" )
print( "Normalized..." )
sclNorm = normalize_dct( sclRead )
pprint( sclNorm )
print( f"Dist. sums to 1.0? {dct_total( sclNorm )}\n" )


##### 3 Updates with Full Reading #########################################
print( "##### Normal Bayes Update #####\n" )
currBel = get_uniform_prior_over_labels()
print( "Init Belief" )
pprint( currBel )
print( f"Dist. sums to 1.0? {dct_total( currBel )}\n" )
for i in range(3):
    currBel = integrate_one_reading( currBel, confMatx, lrgRead, order = _BLOCK_NAMES )
    print( f"Belief after {i+1} readings:\n" )
    pprint( currBel )
    print( f"Dist. sums to 1.0? {dct_total( currBel )}\n" )
print()


##### 3 Updates with Scaled Reading #######################################
print( "##### Bayes Update when Evidence does not sum to 1.0 #####\n" )
currBel = get_uniform_prior_over_labels()
print( "Init Belief" )
pprint( currBel )
print( f"Dist. sums to 1.0? {dct_total( currBel )}\n" )
for i in range(3):
    currBel = integrate_one_reading( currBel, confMatx, sclRead, order = _BLOCK_NAMES )
    print( f"Belief after {i+1} readings:\n" )
    pprint( currBel )
    print( f"Dist. sums to 1.0? {dct_total( currBel )}\n" )
print()

if 0:
    ##### 3 Updates Missing Categories ########################################
    print( "##### Bayes Update when Evidence has zero-value categories #####\n" )
    currBel = get_uniform_prior_over_labels()
    misRead = {
        'redBlock' : 0.0000, 
        'ylwBlock' : 0.0000, 
        'bluBlock' : 0.1665, 
        'grnBlock' : 0.6670, 
        'ornBlock' : 0.1665, 
        'vioBlock' : 0.0000, 
        _NULL_NAME : 0.0000,
    }
    print( "Init Belief" )
    pprint( currBel )
    print( f"Dist. sums to 1.0? {dct_total( currBel )}\n" )
    for i in range(3):
        currBel = integrate_one_reading( currBel, confMatx, misRead, order = _BLOCK_NAMES )
        print( f"Belief after {i+1} readings:\n" )
        pprint( currBel )
        print( f"Dist. sums to 1.0? {dct_total( currBel )}\n" )
    print()


    ##### 3 Updates with Scaled Reading #######################################
    print( "##### Bayes Update when Evidence has zero-value categories and does not sum to 1.0 #####\n" )
    currBel = get_uniform_prior_over_labels()
    bFactor = 0.5
    sclRead = {}
    for label in misRead.keys():
        sclRead[ label ] = misRead[ label ] * bFactor
    print( "Init Belief" )
    pprint( currBel )
    print( f"Dist. sums to 1.0? {dct_total( currBel )}\n" )
    for i in range(3):
        currBel = integrate_one_reading( currBel, confMatx, sclRead, order = _BLOCK_NAMES )
        print( f"Belief after {i+1} readings:\n" )
        pprint( currBel )
        print( f"Dist. sums to 1.0? {dct_total( currBel )}\n" )
    print()