########## INIT ####################################################################################

##### Imports #####

### Standard ###
from pprint import pprint
from ast import literal_eval



########## DATA READ ###############################################################################

def read_symbol_trace( fPath ):
    rtnTrace = dict()
    datList  = list()
    with open( fPath, 'r' ) as f:
        lines = f.readlines()
        accum = ""
        for i, line in enumerate( lines ):
            accum += line

            # print( accum )
            # print()

            try:
                symbols = literal_eval( accum )
                datList.append( symbols )
                accum = ""
            except (ValueError, SyntaxError,):
                pass  
    print( f"There are {len(datList)} items!" ) 
    for datum in datList:
        for sym in datum:
            if sym['label'] not in rtnTrace:
                rtnTrace['label'] = [sym,]
            else:
                rtnTrace['label'].append( sym )
    return rtnTrace



########## DATA PROCESSING #########################################################################

if __name__ == "__main__":
    symData = read_symbol_trace( "data/Sym-Confidence_07-31-2024_14-50-36.txt" )
    print( list(symData.keys()) )