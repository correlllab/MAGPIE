########## INIT ####################################################################################

##### Imports #####

### Standard ###
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
    'grnBlock': { 'plot': 'green' , },
    'bluBlock': { 'plot': 'blue'  , },
    'ylwBlock': { 'plot': 'orange', },
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

    _PLOT_SYM_HISTORY = True
    _VIZ_MEMO_HISTORY = False

    ##### Read File #######################################################

    symData  = read_symbol_trace( "data/Sym-Confidence_08-01-2024_13-09-18.txt" )
    tStart   = symData[0]['time']
    
    synNames = set([])
    for i, datum in enumerate( symData ):
        for sym_j in datum['symbols']:
            synNames.add( sym_j['label'] )

    series = dict()
    series['time'] = list()
    for name in synNames:
        series[ name ] = dict()
        series[ name ]['prob' ] = list()
        series[ name ]['score'] = list()

    ##### Extract Symbol History ##########################################

    for i, datum in enumerate( symData ):
        series['time'].append( datum['time'] - tStart )
        for name in synNames:
            sym_f = None
            for sym_j in datum['symbols']:
                if sym_j['label'] == name:
                    sym_f = sym_j
                    break
            if sym_f is not None:
                # pprint( sym_f )
                series[ name ]['prob' ].append( sym_f['prob' ] )
                series[ name ]['score'].append( sym_f['score'] )
            else:
                series[ name ]['prob' ].append( 0.0 )
                series[ name ]['score'].append( 0.0 )

    if _PLOT_SYM_HISTORY:
        # fig, ax1 = plt.subplots()
        plt.figure()
        

        for name in synNames:
            plt.plot( series['time'], series[ name ]['prob'], _PLOT_TABLE[ name ]['plot'] )
            # plt.plot( series['time'], series[ name ]['score'], _PLOT_TABLE[ name ]['plot'] )

        # ax2 = ax1.twinx()
        # for name in synNames:
        #     ax2.plot( series['time'], series[ name ]['score'], '--' )

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