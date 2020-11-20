'''
Created on Feb 25, 2014

@author: waltj
'''
from pylab import plt
import matplotlib.dates as md
import datetime as dt
from mpl_toolkits.mplot3d import Axes3D
# import time as tm

# import time as systime
# # Profiling code 
# time1 = systime.time()
# # Profiling code 
# time2 = systime.time()
# dtTime = time2 - time1
# print("Profiling time: %.2fs" % (dtTime))

class cObj:
    def __init__(self):
        return

class cPlot:
    def __init__(self):        
        self.sharex = {}    # set to -1 to disable zoom all plots x axis together
        self.f = {}         # dictionary of figures with axes
        self.title = ''
        self.units = ''
        self.preTitle = None
        self.timeIsUtc = 0

    def setPreTitle(self, serNum):
        self.preTitle = serNum
        
    def labels(self, title='', units=''):
        self.title = title;
        self.units = units;

    def plot1(self, figNum, time, data, title='', units='', options=''):
        plt.figure(figNum)
    #     plt.cla()
#         plt.hold(True); 
        plt.grid(True)
        if title:
            self.title = title            
        if not units:
            self.units = units
        if self.preTitle:
            fig = plt.gcf()            
            fig.canvas.set_window_title("Figure %d - %s" % (figNum,self.preTitle))
        plt.title("%s"%(self.title))
        plt.plot(time, data, options)
        plt.ylabel('(%s)'%(self.units))
        plt.xlabel('Time (s)')
        plt.margins(0.04)
    
    
    def plot2(self, figNum, time1, data1, time2, data2, title='', units='', options=''):
        plt.figure(figNum)
#         plt.hold(True); 
        plt.grid(True)
        if title:
            self.title = title            
        if not units:
            self.units = units
    #     plt.cla()
        if self.preTitle:
            fig = plt.gcf()            
            fig.canvas.set_window_title("Figure %d - %s" % (figNum,self.preTitle))
        plt.title("%s"%(self.title))
        plt.plot(time1, data1, options)
        plt.plot(time2, data2, options)
        plt.ylabel('(%s)'%(self.units))
        plt.xlabel('Time (s)')
        plt.margins(0.04)
    
    
    def plot3Axes(self, figNum, time, data, title='', units='', xlabel='Time (s)', options='', xlim=None, ylim=None):
        fig = plt.figure(figNum)
    #     plt.cla()
    
        if title:
            self.title = title            
        if units:
            self.units = units
        if self.preTitle:
            fig.canvas.set_window_title("Figure %d - %s" % (figNum,self.preTitle))
        if not figNum in self.sharex.keys():
            self.sharex[figNum] = plt.subplot(3,1,1)
#             plt.plot(time, data[:,0], options)
                        
        # Plot 1
        subplt = plt.subplot(3,1,1, sharex=self.sharex[figNum])
#         plt.hold(True); 
        plt.grid(True)
        plt.title("%s"%(self.title))
        plt.ylabel('(%s)'%(self.units))
        plt.xlabel(xlabel)
        plt.margins(0.04)
        if xlim:
            subplt.set_xlim(xlim)
        if ylim:
            subplt.set_ylim(ylim)
        if self.timeIsUtc:
            dates=[dt.datetime.fromtimestamp(ts) for ts in time]
            datenums=md.date2num(dates)        
    #         plt.subplots_adjust(bottom=0.2)
            plt.xticks( rotation=25 )
            ax=plt.gca()
            if self.timeIsUtc==2:
                xfmt = md.DateFormatter('%H:%M:%S.%f')
            else:
                xfmt = md.DateFormatter('%H:%M:%S')
            ax.xaxis.set_major_formatter(xfmt)
            plt.plot(datenums, data[:,0], options)
        else:
            plt.plot(time, data[:,0], options)
    
        # Plot 2
        subplt = plt.subplot(3,1,2, sharex=self.sharex[figNum])
#         plt.hold(True); 
        plt.grid(True)
        plt.ylabel('(%s)'%(self.units))
        plt.xlabel(xlabel)
        plt.margins(0.04)
        if xlim:
            subplt.set_xlim(xlim)
        if ylim:
            subplt.set_ylim(ylim)
        if self.timeIsUtc:
            dates=[dt.datetime.fromtimestamp(ts) for ts in time]
            datenums=md.date2num(dates)        
    #         plt.subplots_adjust(bottom=0.2)
            plt.xticks( rotation=25 )
            ax=plt.gca()
            if self.timeIsUtc==2:
                xfmt = md.DateFormatter('%H:%M:%S.%f')
            else:
                xfmt = md.DateFormatter('%H:%M:%S')
            ax.xaxis.set_major_formatter(xfmt)
            plt.plot(datenums, data[:,1], options)
        else:
            plt.plot(time, data[:,1], options)
    
        # Plot 3
        subplt = plt.subplot(3,1,3, sharex=self.sharex[figNum])
#         plt.hold(True); 
        plt.grid(True)
        plt.ylabel('(%s)'%(self.units))
        plt.xlabel(xlabel)
        plt.margins(0.04)
        if xlim:
            subplt.set_xlim(xlim)
        if ylim:
            subplt.set_ylim(ylim)
        if self.timeIsUtc:
            dates=[dt.datetime.fromtimestamp(ts) for ts in time]
            datenums=md.date2num(dates)        
    #         plt.subplots_adjust(bottom=0.2)
            plt.xticks( rotation=25 )
            ax=plt.gca()
            if self.timeIsUtc==2:
                xfmt = md.DateFormatter('%H:%M:%S.%f')
            else:
                xfmt = md.DateFormatter('%H:%M:%S')
            ax.xaxis.set_major_formatter(xfmt)
            plt.plot(datenums, data[:,2], options)
        else:
            plt.plot(time, data[:,2], options)
    #         legend(['desire','actual','e/10','e2/10'])
        return fig

    def plot3setYspan(self, figNum, yspan=None):
        plt.figure(figNum)

        if yspan is None:
            return

        if not figNum in self.sharex.keys():
            self.sharex[figNum] = plt.subplot(3,1,1)
                        
        # Plot 1
        subplt = plt.subplot(3,1,1, sharex=self.sharex[figNum])
        yl = subplt.get_ylim()
        med = (yl[1] - yl[0])*0.5 + yl[0]
        yl = [med-yspan*0.5,med+yspan*0.5]
        subplt.set_ylim(yl)

        # Plot 2
        subplt = plt.subplot(3,1,2, sharex=self.sharex[figNum])
        yl = subplt.get_ylim()
        med = (yl[1] - yl[0])*0.5 + yl[0]
        yl = [med-yspan*0.5,med+yspan*0.5]
        subplt.set_ylim(yl)

        # Plot 3
        subplt = plt.subplot(3,1,3, sharex=self.sharex[figNum])
        yl = subplt.get_ylim()
        med = (yl[1] - yl[0])*0.5 + yl[0]
        yl = [med-yspan*0.5,med+yspan*0.5]
        subplt.set_ylim(yl)

    def plot3D(self, figNum, figTitle=''):
        if not figNum in self.f.keys():
            self.f[figNum] = cObj()
            self.f[figNum].fig = plt.figure()
            self.f[figNum].ax = self.f[figNum].fig.add_subplot(111, projection='3d')
        self.f[figNum].fig.canvas.set_window_title("%s__%s" % (self.preTitle, figTitle))
        return self.f[figNum].fig, self.f[figNum].ax

    def subplots(self, figNum, numRows, figTitle='', sharex=True, numCols=1):
#         if not figNum in self.sharex.keys():
#             self.sharex[figNum] = plt.subplot(numRows,1,plotNum)
# #             plt.plot(time, data, options)
        if not figNum in self.f.keys():
            self.f[figNum] = cObj()
            self.f[figNum].fig, self.f[figNum].ax = plt.subplots(numRows,numCols, sharex=sharex)

        self.f[figNum].fig.canvas.set_window_title("%s__%s" % (self.preTitle, figTitle))

        return self.f[figNum].fig, self.f[figNum].ax
    
    def subplotSingle(self, ax, time, data, title='', ylabel='', xlabel='', options=''):

#         print("subplotSingle ", title, " ", xlabel, " ", ylabel)

        if ylabel:
            self.units = ylabel
                    
#         plt.hold(True); 
        ax.grid(True)
        if title:
            ax.set_title("%s"%(title))
        if xlabel:
            ax.set_xlabel('(%s)'%(xlabel))
        if ylabel:
            ax.set_ylabel('(%s)'%(ylabel))
        ax.margins(0.04)

        if self.timeIsUtc:
            dates=[dt.datetime.fromtimestamp(ts) for ts in time]
            datenums=md.date2num(dates)        
    #         plt.subplots_adjust(bottom=0.2)
            ax.xticks( rotation=25 )
            if self.timeIsUtc==2:
#                 xfmt = md.DateFormatter('%H:%M:%S.%f')
                xfmt = md.DateFormatter('%M:%S.%f')
            else:
#                 xfmt = md.DateFormatter('%H:%M:%S')
                xfmt = md.DateFormatter('%M:%S')
            ax.xaxis.set_major_formatter(xfmt)
            tm = datenums
        else:
            tm = time

        if isinstance(options, str):
            ax.plot(time, data, options)
        else:
            ax.plot(time, data, **options)
                

    def subplotSingle2x(self, figNum, plotNum, numRows, numCols, time, data, title='', units='', options=''):

        print("subplotSingle2x")

        plt.figure(figNum)
        if title:
            self.title = title            
        if not units:
            self.units = units
        if self.preTitle:
            fig = plt.gcf()            
            fig.canvas.set_window_title("%s" % (figNum,self.preTitle))
        if not figNum in self.sharex.keys():
            self.sharex[figNum] = plt.subplot(numRows,numCols,plotNum)
            plt.plot(time, data, options)

        plt.subplot(numRows,numCols,plotNum,sharex=self.sharex[figNum])
#         plt.hold(True); 
        plt.grid(True)
        plt.title("%s"%(self.title))
        plt.plot(time, data, options)
        plt.ylabel('(%s)'%(self.units))
        plt.margins(0.04)
    
    def subplotSetYspan(self, ax, yspan):
        yl = ax.get_ylim()
        med = (yl[1] - yl[0])*0.5 + yl[0]
        yl = [med-yspan*0.5,med+yspan*0.5]
        ax.set_ylim(yl)

    def subplotSetYspanMin(self, ax, yspan):
        yl = ax.get_ylim()
        span = yl[1] - yl[0];
        if span < yspan:
            med = span*0.5 + yl[0]
            yl = [med-yspan*0.5,med+yspan*0.5]
            ax.set_ylim(yl)
    
    def plotNE(self, figNum, north, east, title='', units='', options=''):

        plt.figure(figNum)
    #     plt.cla()
#         plt.hold(True); 
        plt.grid(True)
        if title:
            self.title = title            
        if not units:
            self.units = units
        if self.preTitle:
            fig = plt.gcf()            
            fig.canvas.set_window_title("%s" % (self.preTitle))
        plt.title("%s"%(self.title))
        plt.plot(east, north, options)
        plt.xlabel('East (%s)' %(self.units))
        plt.ylabel('North (%s)'%(self.units))
        
        
