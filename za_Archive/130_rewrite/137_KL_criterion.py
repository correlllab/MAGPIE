########## INIT ####################################################################################
from math import log
from pprint import pprint
import sys
flush = sys.stdout.flush

import numpy as np
import matplotlib.pyplot as plt

from utils import get_confusion_matx, get_confused_class_reading, multiclass_Bayesian_belief_update, roll_outcome
from env_config import _BLOCK_NAMES, _N_CLASSES, _NULL_NAME




########## HELPER FUNCTIONS ########################################################################

def KL_div_info_gain_prior_to_post( priorB, postB ):
    """ Get the discrete KL-Divergence of the posterior from the prior """
    rtnKL = 0.0
    for i in range( len( priorB ) ):
        rtnKL += postB[i] * log( postB[i] / priorB[i] )
    return rtnKL


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


def KL_div_dct( priorBdct, postBdct, order = _BLOCK_NAMES ):
    """ Get the discrete KL-Divergence of the posterior from the prior """
    priorB = extract_class_dist_in_order( priorBdct, order )
    postB  = extract_class_dist_in_order( postBdct , order )
    return KL_div_info_gain_prior_to_post( priorB, postB )


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
    rtnBel = multiclass_Bayesian_belief_update( confMatx, priorB, evidnc )
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


def get_label_noisy( trueLabel, confuseProb, orderedLabels = _BLOCK_NAMES ):
    """ Find one of the ROYGBV blocks, Partially Observable, Return None if the name is not in the world """
    rollDist = get_confused_class_reading( trueLabel, confuseProb, orderedLabels )
    noisyLbl = roll_outcome( rollDist )
    return get_confused_class_reading( noisyLbl, confuseProb, orderedLabels )


########## MAIN ####################################################################################
thrshCrit = 0.55
kldivCrit = 2
N         = 5000

kldivPerf = []
thrshPerf = []
kldivTNs  = []
thrshTNs  = []
probs     = [0.01, 0.025, 0.05, 0.075, 0.10, 0.25, 0.50, 0.55, 0.60, 0.65,]

for conf in probs:

    confProb = conf/6.0
    print( f"\nPer-class confusion: {confProb}", end = "" )
    confMatx = get_confusion_matx( _N_CLASSES, confProb )

    ##### Change in KL-Divergence for Each Update #############################
    # print( "##### Normal Bayes Update #####\n" )
    trueLbl = 'grnBlock'
    initLbl = 'redBlock'
    currBel = get_confused_class_reading( initLbl, 0.10/6.0, _BLOCK_NAMES )
    lastBel = currBel

    # print( "Example Reading" )
    # pprint( get_label_noisy( trueLbl, confProb, orderedLabels = _BLOCK_NAMES ) )
    # print()

    # print( "Init Belief" )
    # pprint( currBel )
    # print()

    kldivStops = []
    thrshStops = []
    kldivCrct  = 0
    thrshCrct  = 0

    for j in range(N):
        kldivLast  = 0
        kldvCount  = 0
        kldivQuit  = False
        thrshQuit  = False
        i          = 0
        

        while not (kldivQuit and thrshQuit):
            i += 1

            nxtRead = get_label_noisy( trueLbl, confProb, orderedLabels = _BLOCK_NAMES )
            currBel = integrate_one_reading( currBel, confMatx, nxtRead, order = _BLOCK_NAMES )

            if not kldivQuit:
                kldivCurr = KL_div_dct( currBel, lastBel, order = _BLOCK_NAMES )
                if kldivCurr < kldivLast:
                    kldvCount += 1
                else:
                    kldvCount = 0
                if kldvCount >= kldivCrit:
                    mxP = 0.0
                    mxL = None
                    for k, v in currBel.items():
                        if v > mxP:
                            mxP = v
                            mxL = k
                    if mxL != initLbl:
                        kldivQuit = True
                        kldivStops.append(i)
                    if mxL == trueLbl:
                        kldivCrct += 1

            if not thrshQuit:
                for k, v in currBel.items():
                    if (v >= thrshCrit) and (k != initLbl):
                        thrshQuit = True
                        thrshStops.append(i)
                        break
                if thrshQuit:
                    mxP = 0.0
                    mxL = None
                    for k, v in currBel.items():
                        if v > mxP:
                            mxP = v
                            mxL = k
                    if mxL == trueLbl:
                        thrshCrct += 1
            

            lastBel   = currBel
            kldivLast = kldivCurr

        if j%100 == 0:
            print( ".", end = "" ) 
            flush()
    
    print()

    kldivMean = np.mean( kldivStops )
    thrshMean = np.mean( thrshStops )

    print( f"Mean steps for KL  detector: {kldivMean}" )
    print( f"Mean steps for {int(thrshCrit*100)}% detector: {thrshMean}" )

    kldivPerf.append( kldivMean )
    thrshPerf.append( thrshMean )
    kldivTNs.append( 1.0 * kldivCrct / N )
    thrshTNs.append( 1.0 * thrshCrct / N )

########## PLOT PERFORMANCE ########################################################################

fig, ax1 = plt.subplots()
ax2 = ax1.twinx()

ax1.plot( probs, kldivPerf, label="KL Step" )
ax1.plot( probs, thrshPerf, label=f"{int(thrshCrit*100)}% Thresh Step" )
ax1.set_ylabel( 'Timestep Halted' )
ax1.set_xlabel('Probability of Class Confusion')

ax2.plot( probs, kldivTNs, 'g', label="KL TP" )
ax2.plot( probs, thrshTNs, 'r', label="Thresh TP" )
ax2.set_ylabel( 'TP' )
ax2.set_ylim([0.75, 1.00])
# ax2.legend( loc=0 )

plt.title('Failure Detector Performance -vs- Confusion')


# ask matplotlib for the plotted objects and their labels
lines, labels = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax2.legend(lines + lines2, labels + labels2, loc=0)

plt.savefig( "DetectorPerf.pdf" )
plt.show()

print( kldivTNs )
print( thrshTNs )