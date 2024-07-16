########## INIT ####################################################################################

import os
from pprint import pprint

import matplotlib.pyplot as plt
import numpy as np

from utils import get_merged_logs_in_dir_with_prefix



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
        dataDct[ name ] = {
            'data'  : data_i,
            'msPass': msPass,
            'msFail': msFail,
        }
    return dataDct



########## PLOTTING ################################################################################

def plot_pass_fail_makespans( msPass, msFail, plotName ):
    plt.boxplot( [msPass, msFail],
                 vert=True,  # vertical box alignment
                 patch_artist=True,  # fill with color
                 labels=["Pass","Fail"],# will be used to label x-ticks
                 showfliers=False)  
    plt.ylabel('Makespan [s]')
    plt.savefig( str( plotName ) + "_pass-fail-makespans" + '.pdf' )

    plt.show()
    plt.clf()


def plot_pass_fail_histo( msPass, msFail, Nbins, plotName ):
    plt.hist( [msPass, msFail], Nbins, histtype='bar', label=["Success", "Failure"] )

    plt.legend(); plt.xlabel('Episode Makespan'); plt.ylabel('Count')
    plt.savefig( str( plotName ) + "_pass-fail-histogram" + '.pdf' )

    plt.show()
    plt.clf()


def plot_sweep_pass_makespans( data, plotName ):
    """ Plot expected successful makespan spread for each category in the sweep """
    datNames = list( data.keys() )
    datNames.sort()
    pltSeries = []
    pltRates  = []
    for datName in datNames:
        pltSeries.append( data[ datName ]['msPass'] )
        pltRates.append( len(data[ datName ]['msPass']) / (len(data[ datName ]['msPass'])+len(data[ datName ]['msFail'])) )

    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()
    
    ax1.boxplot( pltSeries,
                 vert=True,  # vertical box alignment
                 patch_artist=True,  # fill with color
                 labels=datNames,# will be used to label x-ticks
                 showfliers=False)  
    ax1.set_ylabel('Makespan [s]')

    # HACK for misaligned categories, WHY?
    datNames = ['',  ] + datNames
    pltRates = [None,] + pltRates

    # print( datNames, pltRates )
    ax2.plot( datNames, pltRates )
    ax2.set_ylim([0.00, 1.00])
    ax2.set_ylabel('Success Rate')

    plt.title('Block Tower Makespan -vs- Update Frequency, Responsive, 0.30 Confusion')
    ax1.set_xlabel('Seconds Between Belief Updates')
    
    plt.savefig( str( plotName ) + "_sweep-makespans" + '.pdf' )
    plt.show()
    plt.clf()
        




########## MAIN ####################################################################################

if __name__ == "__main__":
    data = collect_multiple_makespan_datasets( 
        {
            f"0.5" : "./data/",
            f"1.0" : "./132a_sweep/data/",
            f"2.0" : "./132b_sweep/data/",
            f"4.0" : "./132c_sweep/data/",
            f"8.0" : "./132d_sweep/data/",
        }, 
        prefix = "TAMP-Loop" 
    )
    plot_sweep_pass_makespans( data, "TAMP-Responsive-Conf" )
    
    
    # get_merged_logs_in_dir_with_prefix( drctry, prefix )
    # print( f"There are {data['N']} trials." )
    # print( f"Success Rate: {data['pass']/data['N']}" )
    
    # msPass, msFail = collect_pass_fail_makespans( data )
    # plot_pass_fail_makespans( msPass, msFail, prefix )
    # plot_pass_fail_histo( msPass, msFail, 10, prefix )
