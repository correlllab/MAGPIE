########## INIT ####################################################################################
from random import random
from random import choice

import numpy as np
import matplotlib.pyplot as plt

from utils import get_merged_logs_in_dir_with_prefix

from env_config import _MIN_X_OFFSET, _BLOCK_SCALE



########## MAKESPAN ESTIMATION #####################################################################


def est_i_baseline( i, t_mv, t_pl, t_rm, P_Ni, N = 1 ):
    """ Estimate serial tower building up to `i` """
    bigSum = prvSum = 0
    for _ in range( N ):
        bigSum += t_mv + t_pl
        bigSum += P_Ni * ( t_rm + prvSum )
        smlSum = 0
        for j in range( 1, i ):
            P_fail_j = 1.0 - ((1.0 - P_Ni) ** (i-j))
            # print( f"Failure Below: {P_fail_j}" )
            smlSum += P_fail_j * t_rm
            for k in range( j, i ):
                smlSum += P_fail_j * (t_rm + est_i_baseline( k, t_mv, t_pl, t_rm, P_Ni, N) )
                # smlSum += P_fail_j * (est_i_baseline( k, t_mv, t_pl, t_rm, P_Ni, N) )
            smlSum += P_fail_j * ( prvSum )
            # smlSum += P_fail_j * ( t_rm + prvSum )
        if i > 1:
            # smlSum /= i-1
            smlSum /= i
        bigSum += smlSum
        prvSum = bigSum
    return bigSum


# def simulate_i_baseline( i, t_mv, t_pl, t_rm, P_Ni, N = 100, mem = None ):
def simulate_tot_baseline( h, t_mv, t_pl, t_rm, P_Ni, N = 100, iterLim = 500 ):
    """ Simulate serial tower building up to `i` """

    _V = False

    T = list()

    blcs = [i+1 for i in range(h)]
    tabl = [i+1 for i in range(h)]
    towr = []

    def identify_blocks_table():
        rtnBlc = []
        for b in tabl:
            if random() < P_Ni:
                rtnBlc.append( choice( [elem for elem in blcs if (elem != b)] ) )
            else:
                rtnBlc.append( b )
        return rtnBlc
    

    def identify_blocks_tower():
        rtnBlc = []
        for b in towr:
            if random() < P_Ni:
                rtnBlc.append( choice( [elem for elem in blcs if (elem != b)] ) )
            else:
                rtnBlc.append( b )
        return rtnBlc

    Nscs = 0

    # For trial in trials
    for _ in range(N):
        tabl = [i+1 for i in range(h)]
        towr = []
        tRun = 0
        rnng = True
        Nitr = 0
        curr = 1
        while rnng:

            Nitr += 1

            tblID = identify_blocks_table()
            twrID = identify_blocks_tower()

            twrOK = (twrID == [i+1 for i in range( len( towr ) )])

            if _V:
                print( f"({twrID})/{twrOK}", end=":" )

            if not twrOK:
                tRun += t_rm
                tabl.append( towr[-1] )
                towr = towr[:-1]
                curr -= 1
                if curr < 1:
                    curr = 1
                if _V:
                    print( towr, end=", " )
                continue
            elif len( towr ) >= h:
                rnng = False
                Nscs += 1
                continue
            
            if curr in tblID:
                popDex = tblID.index( curr )
                popBlc = tabl[ popDex ]
                tRun += t_mv + t_pl
                towr.append( popBlc )
                tabl.pop( popDex )
                curr += 1

                if _V:
                    print( towr, end=", " )
            elif _V:
                print( "DNE", end=", " )

            if Nitr > iterLim:
                rnng = False
        if _V:
            print( towr, end=", " )
            print('\n')
        T.append( tRun )
    print( f"Sim. Success Rate: {Nscs/N}" )
    return np.mean( T )






########## DATA AGGREGATION & PROCESSING ###########################################################

def collect_pass_fail_makespans( data ):
    """ Get the makespans for pass and fail separately so that they can be compared """
    msPass = []
    msFail = []
    for trial in data['trials']:
        if trial['result']:
            msPass.append( trial['makespan'] )
        else:
            msFail.append( trial['makespan'] )
    return msPass, msFail


def collect_multiple_makespan_datasets( dirDct, prefix = "TAMP-Loop" ):
    """ Gather a collection of named datasets for analysis """
    dataDct = {}
    for name, path in dirDct.items():
        data_i = get_merged_logs_in_dir_with_prefix( path, prefix )
        msPass, msFail = collect_pass_fail_makespans( data_i )
        print( f"Series {name}: Success Rate = {len(msPass) / (len(msPass)+len(msFail))}" )
        msTotl = list(); msTotl.extend( msPass ); msTotl.extend( msFail )
        dataDct[ name ] = {
            'data'  : data_i,
            'msPass': msPass,
            'msFail': msFail,
            'msTotl': msTotl,
        }
    return dataDct



########## PLOTTING ################################################################################


def plot_sweep_estimated_makespans( data, plotName, plotTitle ):
    """ Plot expected successful makespan spread for each category in the sweep """
    datNames = list( data.keys() )
    datNames.sort()
    pltMeanM = []
    pltEstMs = []
    pltRates = []
    for datName in datNames:
        pltMeanM.append( data[ datName ]['msMean'] )
        pltEstMs.append( data[ datName ]['msEstm'] )
        pltRates.append( len(data[ datName ]['msPass']) / (len(data[ datName ]['msPass'])+len(data[ datName ]['msFail'])) )

    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()
    
    ax1.plot( datNames,
              pltMeanM,  # vertical box alignment
              label = 'Mean MS' )  
    ax1.plot( datNames,
              pltEstMs,  # vertical box alignment
              label = 'Est. MS' )  
    ax1.set_ylabel('Makespan [s]')
    ax1.legend( ['Mean MS','Est. MS',] )

    # HACK for misaligned categories, WHY?
    datNames = ['',  ] + datNames
    pltRates = [None,] + pltRates

    # print( datNames, pltRates )
    ax2.plot( datNames, pltRates )
    ax2.set_ylim([0.00, 1.00])
    ax2.set_ylabel('Success Rate')
    

    # plt.title('Block Tower Makespan -vs- Update Frequency, Responsive, 0.30 Confusion')
    plt.title( plotTitle )
    ax1.set_xlabel('Confusion')
    
    plt.savefig( str( plotName ) + "_sweep-makespans" + '.pdf' )
    plt.show()
    plt.clf()


########## GET DATA ################################################################################
_EXCLUDE_PDLS = False

# baseDir = "data/"
# prefix  = "TAMP-Loop"
# data_i  = get_merged_logs_in_dir_with_prefix( baseDir, prefix )


if __name__ == "__main__":

    if 0:
        simulate_tot_baseline( 4, 1, 1, 2, 0.45, N = 1000 )
    else:

        data = collect_multiple_makespan_datasets( 
            {
                f"{np.round(0.001*6.0,3)}" : "./data/",
                f"{np.round(0.01*6.0,3)}"  : "./132a_sweep/data/",
                f"{np.round(0.025*6.0,3)}" : "./132b_sweep/data/",
                f"{np.round(0.05*6.0,3)}"  : "./132c_sweep/data/",
                f"{np.round(0.075*6.0,3)}" : "./132d_sweep/data/",
            }, 
            prefix = "TAMP-Loop" 
        )
        for sName, data_i in data.items():

            data_ii = data_i['data']

            if not _EXCLUDE_PDLS:
                mkSpans  = data_i['msTotl']
                avgMkspn = np.mean( mkSpans )
            else:
                mkSpans  = []
                avgMkspn = 0.0

            print( f"There are {len(data_ii['trials'])} recorded trials for {sName} Confusion" )

            # print( data_i['trials'][0].keys() )

            t_mv_all = list()
            t_pl_all = list()
            t_rm_all = list()
            N_bktrk  = 0 # Number of times we backtracked
            N_blcPln = 0 # Total of all block move plans
            target   = [ _MIN_X_OFFSET+6.0*_BLOCK_SCALE, 0.000, 1.0*_BLOCK_SCALE,  1,0,0,0 ]

            # For every trial in the series
            for trial in data_ii['trials']:

                ms_i = 0

                if not trial['result']:
                    continue

                # Setup timing vars
                t_mv_bgn = 0.0
                t_pl_bgn = 0.0
                t_rm_bgn = 0.0
                t_mv_i   = 0.0
                t_pl_i   = 0.0
                t_rm_i   = 0.0
                moved    = set(list())
                start    = False
                bgnMv    = False
                p_mstk   = False
                
                # For every event in the trial, Extract elems, compute timing, and determine backtracking
                for (time_e, name_e, mesg_e) in trial['events']:

                    # print( time_e, name_e, mesg_e )

                    # Handle beginning and end
                    if (name_e == "Begin Solver"):  # 2024-05-31: For now, Include PDLS in the move time
                    # if (name_e == "Begin Solver") or ("BT END" in name_e):  # 2024-05-31: For now, Include PDLS in the move time
                    # if ("Move_" in name_e):  
                        if not start:
                            N_blcPln += 1
                            t_mv_bgn = time_e
                            start    = True
                    
                    # elif (name_e == "Begin Solver"):  # 2024-05-31: For now, Include PDLS in the move time
                    # elif ("BT END" in name_e):  
                        if bgnMv:
                        # else:
                            if p_mstk:
                                t_rm_i = time_e - t_rm_bgn
                                t_rm_all.append( t_rm_i )
                                if _EXCLUDE_PDLS:
                                    ms_i += t_rm_i
                            else:
                                if (t_pl_bgn > 0.0):
                                    t_pl_i = time_e - t_pl_bgn
                                if (t_pl_i > 0.0):
                                    t_mv_all.append( t_mv_i )
                                    t_pl_all.append( t_pl_i )
                                    if _EXCLUDE_PDLS:
                                        ms_i += t_mv_i + t_pl_i
                            t_mv_i   = 0.0
                            t_pl_i   = 0.0
                            t_rm_i   = 0.0
                            t_mv_bgn = 0.0
                            t_pl_bgn = 0.0
                            t_rm_bgn = 0.0
                            p_mstk   = False
                            start    = False
                            bgnMv    = False

                    elif ("Move_" in name_e):  
                        if start:
                            bgnMv = True

                        
                    
                    elif ("Pick" in name_e) or ("Unstack" in name_e):

                        coords = (name_e.replace('[','|').replace(']','|').split('|')[1]).split()
                        # print( f"{coords}, {len(coords)}" )
                        moveTo = [float(c) for c in coords]
                        diffVc = np.subtract( moveTo, target )

                        if np.linalg.norm( diffVc[:2] ) < _BLOCK_SCALE:
                            p_mstk   = True
                            t_rm_bgn = t_mv_bgn
                            N_bktrk  += 1

                    # elif ("Pick" in name_e):
                        if not p_mstk:
                            # Compute move && Begin place
                            t_mv_i   = time_e - t_mv_bgn
                            t_pl_bgn = time_e
                            # Log the block we are moving: In the ideal case we only move each block *once*
                            # parts = name_e.split()
                            # for elem in parts:
                            #     if "Block" in elem:
                            #         if elem in moved:
                                        
                            #         moved.add( elem )

                if _EXCLUDE_PDLS:
                    mkSpans.append( ms_i )
                        
            if _EXCLUDE_PDLS:
                avgMkspn = np.mean( mkSpans )

            t_mv_avg = np.mean( t_mv_all )
            t_pl_avg = np.mean( t_pl_all )
            t_rm_avg = np.mean( t_rm_all )
            # t_rm_avg = t_mv_avg + t_pl_avg # 2024-05-31: For now, assume re/move take the same amount of time
            
            # P_repeat = N_bktrk/N_blcPln
            P_repeat = float(sName)

            print( f"\n\nCounted {len(t_mv_all)},{len(t_pl_all)} per-block plans." )
            print( f"Mean t_mv: {t_mv_avg}" )
            print( f"Mean t_pl: {t_pl_avg}" )
            print( f"Mean t_rm: {t_rm_avg}" )
            # print( f"Backtrack Rate: {P_repeat}" )
            print( f"Backtrack Rate: {float(sName)}" )
            print( f"Average Makespan across all trials: {avgMkspn}" )


            # totalMkspn4bloc  = simulate_i_baseline( 4, t_mv_avg, t_pl_avg, t_rm_avg, P_repeat )
            # totalMkspn4bloc  = simulate_tot_baseline( 4, t_mv_avg, t_pl_avg, t_rm_avg, float(sName), 10000 )
            # totalMkspn4bloc  = simulate_tot_baseline( 4, t_mv_avg, t_pl_avg, t_rm_avg, P_repeat, 10000 )
            totalMkspn4bloc = est_i_baseline( 4, t_mv_avg, t_pl_avg, t_rm_avg, P_repeat, N = 1 )
            data_i['msEstm'] = totalMkspn4bloc
            data_i['msMean'] = avgMkspn

            print( f"Estimated Makespan across all trials: {totalMkspn4bloc}\n\n#####################" )

        plot_sweep_estimated_makespans( 
            data, 
            "TAMP-Baseline-Est", 
            "Block Tower Makespan -vs- Confusion, Baseline, Estimated", 
        )