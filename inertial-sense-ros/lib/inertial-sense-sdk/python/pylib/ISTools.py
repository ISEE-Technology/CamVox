import json
import os
import numpy as np
import time as systime
from pylab import plt
from sys import platform

import pylib.ISToolsDataSorted as itd
import pylib.ISToolsPlot as itp
import pylib.ISDataAnalytics as ida
import pylib.newISDataAnalytics as newida

class cObj:
    def __init__(self):
        return


def plotdata(opt, pe, settings_file):

    log = None

    # if opt['use_settings_file']:
    if settings_file is not None:
        with open(settings_file) as data_file:
            data = json.load(data_file)
            directory = data['dataInfo']['dataDirectory']
            subdir = data['processData']['datasets'][0]['folder']
            serialNumbers =  data['processData']['datasets'][0]['SerialNumbers']
    else:
        directory = opt['directory']
        serialNumbers = opt['serials']
        subdir = ""
    print("Directory: %s" % directory)
    print("Folder: %s" % subdir)
    print("Serial Numbers: %s" % serialNumbers)
    # else:
    #     directory = opt['directory']
    #     serialNumbers = opt['serials']
    #     subdir = None
    #
    if subdir:
        directory += subdir
        
    # Set Reference LLA (deg, deg, m) used for NED - Salem, UT   
    itd.setRefLla( np.r_[ 40.0557114, -111.6585476, 1426.77 ] )
    
    if pe['postProcess']==1:
        # Temporarily Turn off certain plots for reference
        googleEarthTemp = pe['googleEarth']
        pe['googleEarth'] = 0
    
    if opt['showReference'] or pe['postProcess']==0:
        # Load Reference INS Data
        log = _load(directory, opt)
        
    if pe['postProcess']==1:
        directory += '/post_processed'
        pe['googleEarth'] = googleEarthTemp
    
        # Load Reference INS Data
        log = _load(directory, opt)

    # Check Raw GPS observation counts
    if opt['checkRawDataDrop']:
        ida.checkRawDataDrop(log)

    # Turn on plots
    if opt['showFigs']:
        plotlog(log, directory, serialNumbers, opt, pe)

    # RMS Accuracies
    if opt['rmsCalc'] and not opt['showReference']:
        newida.calcRMS(log, directory, subdir)

    return (log, directory)


def _load(directory, opt):
    
    cwd = os.getcwd()
    
    try:
        tru = itd.cDevices()
        tru.loadData(refIns=1)
    except:
        tru = None

    # Load Log Data
    log = itd.cDevices()
    log.loadData(directory, startDev=opt['start'], devCount=opt['count'])

    os.chdir(cwd)
    
    return log


def plotlog(log, directory, serialNumbers, opt, pe):
    
    # Profiling
    timePlotStart = systime.time()

#     basename = os.path.basename(directory)
#     dst = os.path.join(directory,subdir)
    dst = os.path.join(directory,"figures")
    if pe['postProcess']==1:
        dst = os.path.join(dst,'post_processed')
    if opt['saveFigs']:
        if not os.path.exists(dst):
            os.makedirs(dst)    

    # Plot Data
    figNum = 0
    for device2 in log.devices:
        if device2.serialNumber not in serialNumbers and not "ALL" in serialNumbers and serialNumbers != []:
            continue

        referencePlot = False        
#         if pe['postProcess==1:
#             referencePlot = True    # Color reference plots one color
            
        f = itp.IsLoggerPlot( pe, device2, startFigure=figNum, saveFigs=opt['saveFigs'], saveFigsDirectory=dst, referencePlot=referencePlot, numDevs=len(log.devices))

        if opt['combinePlots']==0:
            figNum += f        
    print("Total times:    Load %.2fs     Plot %.2fs" % (log.loadTime, systime.time()-timePlotStart))
        
                
def end():
    plt.show()  # Do this last
 
