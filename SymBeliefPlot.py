########## INIT ####################################################################################

##### Imports #####

### Standard ###
import os
from pprint import pprint
from ast import literal_eval

### Special ###
import matplotlib.pyplot as plt
import numpy as np
from vispy import gloo, app

### Local ###
from graphics.draw_beliefs import reading_dict_geo, vispy_geo_list_window, table_geo, _TABLE_THIC
from env_config import ( _LKG_SEP, _MIN_X_OFFSET, _MIN_Y_OFFSET, _X_WRK_SPAN, _Y_WRK_SPAN, )


_LOOK = [ _MIN_X_OFFSET + _X_WRK_SPAN/2.0, 
          _MIN_Y_OFFSET + _Y_WRK_SPAN/2.0, 
          -_TABLE_THIC/2.0 ]
_CAM_POS = np.add( _LOOK, [ 0.5, 0.5, 0.5 ] )


########## DATA READ ###############################################################################
_PLOT_TABLE = {
    # Confidence Plots
    'grnBlock': { 'plot': 'green' , 'name': 'Green Block' },
    'bluBlock': { 'plot': 'blue'  , 'name': 'Blue Block' },
    'ylwBlock': { 'plot': 'orange', 'name': 'Yellow Block' },
    # Memory Rendering
    'symbol'  : 'black',
    'LKG'     : 'red',
    'belief'  : 'yellow',
    'obs'     : 'white',
}

def read_symbol_trace( fPath ):
    datList  = list()
    with open( fPath, 'r' ) as f:
        lines = f.readlines()
        for _, line in enumerate( lines ):
            symbols = literal_eval( line )
            datList.append( symbols )
    print( f"There are {len(datList)} items!" ) 
    return datList


def get_reading_list_geo( readings, kind = 'symbol' ):
    geo  = list()
    txt  = list()
    sMax = -1e6
    
    if kind in _PLOT_TABLE:
        outClr = _PLOT_TABLE[ kind ]
    else:
        outClr = _PLOT_TABLE['symbol']

    for lkg_i in readings:
        if lkg_i['score'] > sMax:
            sMax = lkg_i['score']

    for lkg_i in readings:
        print( lkg_i['score'] )
        alpha = max( 0.00, lkg_i['score']/sMax )
        geo_i, txt_i = reading_dict_geo( lkg_i, lineColor = outClr, baseAlpha = alpha )
        geo.append( geo_i )
        txt.append( txt_i )
    return geo, txt



########## Visualization Functions #################################################################





########## DATA PROCESSING #########################################################################



if __name__ == "__main__":

    _PLOT_SYM_HISTORY = 1
    _VIZ_MEMO_HISTORY = 0
    _OTHER_FIGURES    = 0
    _SHOW_PLOTS       = 0

    ##### Read File #######################################################

    symData  = read_symbol_trace( "data/Sym-Confidence_08-01-2024_13-09-18.txt" )
    tStart   = symData[0]['time']
    
    synNames = set([])
    for i, datum in enumerate( symData ):
        for sym_j in datum['symbols']:
            synNames.add( sym_j['label'] )

    series = dict()
    series['time'   ] = list()
    series['MaxLike'] = list()
    for name in synNames:
        series[ name ] = dict()
        series[ name ]['prob' ] = list()
        series[ name ]['score'] = list()

    ##### Extract Symbol History ##########################################

    for i, datum in enumerate( symData ):
        series['time'].append( datum['time'] - tStart )
        mxLk_i = 1.0
        for name in synNames:
            sym_f = None
            for sym_j in datum['symbols']:
                if sym_j['label'] == name:
                    sym_f = sym_j
                    break
            if sym_f is not None:
                mxLk_i *= sym_f['prob' ]
                series[ name ]['prob' ].append( sym_f['prob' ] )
                series[ name ]['score'].append( sym_f['score'] )
            else:
                series[ name ]['prob' ].append( 0.0 )
                series[ name ]['score'].append( 0.0 )
        series['MaxLike'].append( mxLk_i )

    if _PLOT_SYM_HISTORY:
        
        plt.rcParams.update({
            "text.usetex": True,
        })

        
        
        if 0:
            plt.figure( figsize = (18,18), dpi = 300 )
            for name in synNames:
                # plt.plot( series['time'], series[ name ]['prob'], _PLOT_TABLE[ name ]['plot'] )
                plt.plot( series['time'], series[ name ]['score'], _PLOT_TABLE[ name ]['plot'], 
                        label = _PLOT_TABLE[ name ]['name'], linewidth = 5 )
            plt.title( 'Symbol Quality $s_{obj}$ -vs- Time', fontsize = 60 )
            plt.xlabel( 'Time [s]', fontsize = 40 )
            plt.ylabel( 'Quality $s_{obj}$', fontsize = 40 )
            plt.legend( fontsize = 40 )
            plt.xticks( fontsize = 30 )
            plt.yticks( fontsize = 30 )
            plt.savefig( 'graphics/paper/QualityPlot.pdf' )

        elif 1:

            fig, axs = plt.subplots( 2, figsize = (12,16), dpi = 300 )
            # fig.suptitle( 'Vertically stacked subplots' )
            

            ### Top Plot ###

            for name in synNames:
                axs[0].plot( series['time'], series[ name ]['score'], _PLOT_TABLE[ name ]['plot'], 
                             label = _PLOT_TABLE[ name ]['name'], linewidth = 5 )
            
            axs[0].set_title('Axis [0, 0]', fontsize = 30 )
            axs[0].legend( fontsize = 20 )
            axs[0].tick_params( axis='both', labelsize = 15 )

            ### Bottom Plot ###

            axs[1].set_title( 'Axis [0, 0]', fontsize = 30 )
            axs[1].plot( series['time'], series['MaxLike'], linewidth = 5 )
            axs[1].tick_params( axis='both', labelsize = 15 )

            plt.tight_layout( pad   = 2.25, 
                              w_pad = 0, 
                              h_pad = 1.25 )

            plt.savefig( 'graphics/paper/QualityPlot_V2.pdf' )

        
        if _SHOW_PLOTS: 
            plt.show()
        plt.clf()
        

    ##### Extract Symbol Decision Making ##################################
    
    if _VIZ_MEMO_HISTORY:

        for datum in symData[:1]:
            symCubes = [table_geo(),]
            geo, txt = get_reading_list_geo( datum['LKGmem'], 'LKG' )
            symCubes.extend( geo )
            symCubes.extend( txt )
            # symCubes.extend( get_reading_list_geo( datum['beliefs'] ) )
            vispy_geo_list_window( symCubes, )


    ##### Various Calcs and Figs ##########################################

    if _OTHER_FIGURES:
        from TaskPlanner import entropy_factor

        print( (1.0-entropy_factor( [0.75,0.25,0.0,0.0]     )) * 4 )
        print( (1.0-entropy_factor( [0.5,0.25,0.25,0.0]     )) * 4 )
        print( (1.0-entropy_factor( [0.333,0.333,0.333,0.0] )) * 3 )

    os.system( 'kill %d' % os.getpid() ) 