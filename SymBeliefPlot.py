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
from magpie.poses import translation_diff
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
    """ Obtain geometry """
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


def p_orig_pose( qXfrm ):
    """ Return True if `qXfrm` is equal to the origin pose """
    try:
        return (translation_diff( np.array( qXfrm ), np.eye(4) ) < _LKG_SEP)
    except Exception:
        return False





########## DATA PROCESSING #########################################################################



if __name__ == "__main__":

    _PLOT_SYM_HISTORY = 1
    _VIZ_MEMO_HISTORY = 0
    _OTHER_FIGURES    = 0
    _SHOW_PLOTS       = 0

    _TITLE_FSIZ  = 25
    _LEGEND_FSIZ = 20
    _TICK_FSIZ   = 15
    _AXES_FSIZ   = 25
    _LINE_WIDTH  =  5
    _PLOT_ENDEX  = -9

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
    series['SymMove'] = list()
    for name in synNames:
        series[ name ] = dict()
        series[ name ]['prob' ] = list()
        series[ name ]['score'] = list()
        series[ name ]['pose' ] = list()

    ##### Extract Symbol History ##########################################

    for i, datum in enumerate( symData ):
        series['time'].append( datum['time'] - tStart )
        mxLk_i   = 1.0
        deltaSym = False
        transMax = 0.0
        nameDmax = None
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
                series[ name ]['pose' ].append( np.array( sym_f['pose'] ) )
            else:
                series[ name ]['prob' ].append( 0.0 )
                series[ name ]['score'].append( 0.0 )
                series[ name ]['pose' ].append( np.eye(4) )
            
            if (i > 1):
                nameDist = translation_diff( series[ name ]['pose' ][-1], series[ name ]['pose' ][-2] )
                if (nameDist > _LKG_SEP*3.0) and (not p_orig_pose( series[ name ]['pose' ][-1] )):
                    deltaSym = True
                    if nameDist > transMax:
                        transMax = nameDist
                        nameDmax = name


        if deltaSym:
            series['SymMove'].append( nameDmax )
        else:
            series['SymMove'].append( 0 )
        
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
                        label = _PLOT_TABLE[ name ]['name'], linewidth = _LINE_WIDTH )
            plt.title( 'Symbol Quality $s_{obj}$ -vs- Time', fontsize = 60 )
            plt.xlabel( 'Time [s]', fontsize = 40 )
            plt.ylabel( 'Quality $s_{obj}$', fontsize = 40 )
            plt.legend( fontsize = 40 )
            plt.xticks( fontsize = 30 )
            plt.yticks( fontsize = 30 )
            plt.savefig( 'graphics/paper/QualityPlot.pdf' )

        elif 1:

            fig, axs = plt.subplots( nrows=1, ncols=2, figsize = (24,8), dpi = 300 )
            # fig.suptitle( 'Vertically stacked subplots' )
            

            ### Top Plot ###

            for name in synNames:
                axs[0].plot( series['time'][:_PLOT_ENDEX], 
                             series[ name ]['score'][:_PLOT_ENDEX], 
                             _PLOT_TABLE[ name ]['plot'], 
                             label = _PLOT_TABLE[ name ]['name'], linewidth = _LINE_WIDTH )
            
            axs[0].set_title('Symbol Quality $s_{obj}$ -vs- Time', fontsize = _TITLE_FSIZ )
            axs[0].legend( fontsize = _LEGEND_FSIZ )
            axs[0].tick_params( axis='both', labelsize = _TICK_FSIZ )
            axs[0].set_xlabel( 'Time [s]', fontsize = _AXES_FSIZ )
            axs[0].set_ylabel( 'Quality $s_{obj}$', fontsize = _AXES_FSIZ )

            ### Bottom Plot ###

            axs[1].set_title( 'Maximum Likelihood of Grounded Symbols -vs- Time', fontsize = _TITLE_FSIZ )
            axs[1].plot( series['time'][:_PLOT_ENDEX], 
                         series['MaxLike'][:_PLOT_ENDEX], 
                         linewidth = _LINE_WIDTH, color = 'purple' )
            axs[1].tick_params( axis='both', labelsize = _TICK_FSIZ )
            axs[1].set_xlabel( 'Time [s]', fontsize = _AXES_FSIZ )
            axs[1].set_ylabel( 'Max. Likelihood', fontsize = _AXES_FSIZ )
            for i, p_move_i in enumerate( series['SymMove'][:_PLOT_ENDEX] ):
                if p_move_i:
                    axs[1].axvline( x = series['time'][i], color = _PLOT_TABLE[ p_move_i ]['plot'] )

            plt.tight_layout( pad   = 2.25, 
                              w_pad = 1.25, 
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