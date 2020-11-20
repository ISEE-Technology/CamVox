##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=CLTool
ConfigurationName      :=Debug
WorkspacePath          := "/home/pi/share/IS-src/cpp/SDK/CLTool/CodeLite_project"
ProjectPath            := "/home/pi/share/IS-src/cpp/SDK/CLTool/CodeLite_project"
IntermediateDirectory  :=./Debug
OutDir                 := $(IntermediateDirectory)
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=
Date                   :=10/24/17
CodeLitePath           :="/home/pi/.codelite"
LinkerName             :=/usr/bin/g++ 
SharedObjectLinkerName :=/usr/bin/g++ -shared -fPIC
ObjectSuffix           :=.o
DependSuffix           :=.o.d
PreprocessSuffix       :=.i
DebugSwitch            :=-g 
IncludeSwitch          :=-I
LibrarySwitch          :=-l
OutputSwitch           :=-o 
LibraryPathSwitch      :=-L
PreprocessorSwitch     :=-D
SourceSwitch           :=-c 
OutputFile             :=$(IntermediateDirectory)/$(ProjectName)
Preprocessors          :=
ObjectSwitch           :=-o 
ArchiveOutputSwitch    := 
PreprocessOnlySwitch   :=-E
ObjectsFileList        :="CLTool.txt"
PCHCompileFlags        :=
MakeDirCommand         :=mkdir -p
LinkOptions            :=  
IncludePath            :=  $(IncludeSwitch). $(IncludeSwitch). 
IncludePCH             := 
RcIncludePath          := 
Libs                   := $(LibrarySwitch)pthread 
ArLibs                 :=  "pthread" 
LibPath                := $(LibraryPathSwitch). 

##
## Common variables
## AR, CXX, CC, AS, CXXFLAGS and CFLAGS can be overriden using an environment variables
##
AR       := /usr/bin/ar rcu
CXX      := /usr/bin/g++ 
CC       := /usr/bin/gcc 
CXXFLAGS :=  -g -O0 -Wall -std=gnu++11 $(Preprocessors)
CFLAGS   :=  -g -O0 -Wall -std=gnu11 $(Preprocessors)
ASFLAGS  := 
AS       := /usr/bin/as 


##
## User defined environment variables
##
CodeLiteDir:=/usr/share/codelite
Objects0=$(IntermediateDirectory)/source_main.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_DataCSV.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_DataChunk.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_DataChunkSorted.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_DataKML.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_DeviceLog.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_DeviceLogCSV.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_DeviceLogKML.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_DeviceLogSerial.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_DeviceLogSorted.cpp$(ObjectSuffix) \
	$(IntermediateDirectory)/src_GpsParser.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_ISDataMappings.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_ISDisplay.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_ISLogger.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_ISMatrix.c$(ObjectSuffix) $(IntermediateDirectory)/src_ISPose.c$(ObjectSuffix) $(IntermediateDirectory)/src_ISTcpClient.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_ISTcpServer.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_ISUtilities.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_InertialSense.cpp$(ObjectSuffix) \
	$(IntermediateDirectory)/src_cltool.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_cltool_main.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_com_manager.c$(ObjectSuffix) $(IntermediateDirectory)/src_data_sets.c$(ObjectSuffix) $(IntermediateDirectory)/src_inertialSenseBootLoader.c$(ObjectSuffix) $(IntermediateDirectory)/src_linked_list.c$(ObjectSuffix) $(IntermediateDirectory)/src_serialPort.c$(ObjectSuffix) $(IntermediateDirectory)/src_serialPortPlatform.c$(ObjectSuffix) $(IntermediateDirectory)/src_time_conversion.c$(ObjectSuffix) $(IntermediateDirectory)/src_tinystr.cpp$(ObjectSuffix) \
	$(IntermediateDirectory)/src_tinyxml.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_tinyxmlerror.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_tinyxmlparser.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_ISSerialPort.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_ISStream.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_ISEarth.c$(ObjectSuffix) 



Objects=$(Objects0) 

##
## Main Build Targets 
##
.PHONY: all clean PreBuild PrePreBuild PostBuild
all: $(OutputFile)

$(OutputFile): $(IntermediateDirectory)/.d $(Objects) 
	@$(MakeDirCommand) $(@D)
	@echo "" > $(IntermediateDirectory)/.d
	@echo $(Objects0)  > $(ObjectsFileList)
	$(LinkerName) $(OutputSwitch)$(OutputFile) @$(ObjectsFileList) $(LibPath) $(Libs) $(LinkOptions)

$(IntermediateDirectory)/.d:
	@test -d ./Debug || $(MakeDirCommand) ./Debug

PreBuild:


##
## Objects
##
$(IntermediateDirectory)/source_main.cpp$(ObjectSuffix): ../source/main.cpp $(IntermediateDirectory)/source_main.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/CLTool/source/main.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/source_main.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/source_main.cpp$(DependSuffix): ../source/main.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/source_main.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/source_main.cpp$(DependSuffix) -MM "../source/main.cpp"

$(IntermediateDirectory)/source_main.cpp$(PreprocessSuffix): ../source/main.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/source_main.cpp$(PreprocessSuffix) "../source/main.cpp"

$(IntermediateDirectory)/src_DataCSV.cpp$(ObjectSuffix): ../../src/DataCSV.cpp $(IntermediateDirectory)/src_DataCSV.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/DataCSV.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_DataCSV.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_DataCSV.cpp$(DependSuffix): ../../src/DataCSV.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_DataCSV.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_DataCSV.cpp$(DependSuffix) -MM "../../src/DataCSV.cpp"

$(IntermediateDirectory)/src_DataCSV.cpp$(PreprocessSuffix): ../../src/DataCSV.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_DataCSV.cpp$(PreprocessSuffix) "../../src/DataCSV.cpp"

$(IntermediateDirectory)/src_DataChunk.cpp$(ObjectSuffix): ../../src/DataChunk.cpp $(IntermediateDirectory)/src_DataChunk.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/DataChunk.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_DataChunk.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_DataChunk.cpp$(DependSuffix): ../../src/DataChunk.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_DataChunk.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_DataChunk.cpp$(DependSuffix) -MM "../../src/DataChunk.cpp"

$(IntermediateDirectory)/src_DataChunk.cpp$(PreprocessSuffix): ../../src/DataChunk.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_DataChunk.cpp$(PreprocessSuffix) "../../src/DataChunk.cpp"

$(IntermediateDirectory)/src_DataChunkSorted.cpp$(ObjectSuffix): ../../src/DataChunkSorted.cpp $(IntermediateDirectory)/src_DataChunkSorted.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/DataChunkSorted.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_DataChunkSorted.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_DataChunkSorted.cpp$(DependSuffix): ../../src/DataChunkSorted.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_DataChunkSorted.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_DataChunkSorted.cpp$(DependSuffix) -MM "../../src/DataChunkSorted.cpp"

$(IntermediateDirectory)/src_DataChunkSorted.cpp$(PreprocessSuffix): ../../src/DataChunkSorted.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_DataChunkSorted.cpp$(PreprocessSuffix) "../../src/DataChunkSorted.cpp"

$(IntermediateDirectory)/src_DataKML.cpp$(ObjectSuffix): ../../src/DataKML.cpp $(IntermediateDirectory)/src_DataKML.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/DataKML.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_DataKML.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_DataKML.cpp$(DependSuffix): ../../src/DataKML.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_DataKML.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_DataKML.cpp$(DependSuffix) -MM "../../src/DataKML.cpp"

$(IntermediateDirectory)/src_DataKML.cpp$(PreprocessSuffix): ../../src/DataKML.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_DataKML.cpp$(PreprocessSuffix) "../../src/DataKML.cpp"

$(IntermediateDirectory)/src_DeviceLog.cpp$(ObjectSuffix): ../../src/DeviceLog.cpp $(IntermediateDirectory)/src_DeviceLog.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/DeviceLog.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_DeviceLog.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_DeviceLog.cpp$(DependSuffix): ../../src/DeviceLog.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_DeviceLog.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_DeviceLog.cpp$(DependSuffix) -MM "../../src/DeviceLog.cpp"

$(IntermediateDirectory)/src_DeviceLog.cpp$(PreprocessSuffix): ../../src/DeviceLog.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_DeviceLog.cpp$(PreprocessSuffix) "../../src/DeviceLog.cpp"

$(IntermediateDirectory)/src_DeviceLogCSV.cpp$(ObjectSuffix): ../../src/DeviceLogCSV.cpp $(IntermediateDirectory)/src_DeviceLogCSV.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/DeviceLogCSV.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_DeviceLogCSV.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_DeviceLogCSV.cpp$(DependSuffix): ../../src/DeviceLogCSV.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_DeviceLogCSV.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_DeviceLogCSV.cpp$(DependSuffix) -MM "../../src/DeviceLogCSV.cpp"

$(IntermediateDirectory)/src_DeviceLogCSV.cpp$(PreprocessSuffix): ../../src/DeviceLogCSV.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_DeviceLogCSV.cpp$(PreprocessSuffix) "../../src/DeviceLogCSV.cpp"

$(IntermediateDirectory)/src_DeviceLogKML.cpp$(ObjectSuffix): ../../src/DeviceLogKML.cpp $(IntermediateDirectory)/src_DeviceLogKML.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/DeviceLogKML.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_DeviceLogKML.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_DeviceLogKML.cpp$(DependSuffix): ../../src/DeviceLogKML.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_DeviceLogKML.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_DeviceLogKML.cpp$(DependSuffix) -MM "../../src/DeviceLogKML.cpp"

$(IntermediateDirectory)/src_DeviceLogKML.cpp$(PreprocessSuffix): ../../src/DeviceLogKML.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_DeviceLogKML.cpp$(PreprocessSuffix) "../../src/DeviceLogKML.cpp"

$(IntermediateDirectory)/src_DeviceLogSerial.cpp$(ObjectSuffix): ../../src/DeviceLogSerial.cpp $(IntermediateDirectory)/src_DeviceLogSerial.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/DeviceLogSerial.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_DeviceLogSerial.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_DeviceLogSerial.cpp$(DependSuffix): ../../src/DeviceLogSerial.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_DeviceLogSerial.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_DeviceLogSerial.cpp$(DependSuffix) -MM "../../src/DeviceLogSerial.cpp"

$(IntermediateDirectory)/src_DeviceLogSerial.cpp$(PreprocessSuffix): ../../src/DeviceLogSerial.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_DeviceLogSerial.cpp$(PreprocessSuffix) "../../src/DeviceLogSerial.cpp"

$(IntermediateDirectory)/src_DeviceLogSorted.cpp$(ObjectSuffix): ../../src/DeviceLogSorted.cpp $(IntermediateDirectory)/src_DeviceLogSorted.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/DeviceLogSorted.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_DeviceLogSorted.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_DeviceLogSorted.cpp$(DependSuffix): ../../src/DeviceLogSorted.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_DeviceLogSorted.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_DeviceLogSorted.cpp$(DependSuffix) -MM "../../src/DeviceLogSorted.cpp"

$(IntermediateDirectory)/src_DeviceLogSorted.cpp$(PreprocessSuffix): ../../src/DeviceLogSorted.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_DeviceLogSorted.cpp$(PreprocessSuffix) "../../src/DeviceLogSorted.cpp"

$(IntermediateDirectory)/src_GpsParser.cpp$(ObjectSuffix): ../../src/GpsParser.cpp $(IntermediateDirectory)/src_GpsParser.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/GpsParser.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_GpsParser.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_GpsParser.cpp$(DependSuffix): ../../src/GpsParser.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_GpsParser.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_GpsParser.cpp$(DependSuffix) -MM "../../src/GpsParser.cpp"

$(IntermediateDirectory)/src_GpsParser.cpp$(PreprocessSuffix): ../../src/GpsParser.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_GpsParser.cpp$(PreprocessSuffix) "../../src/GpsParser.cpp"

$(IntermediateDirectory)/src_ISDataMappings.cpp$(ObjectSuffix): ../../src/ISDataMappings.cpp $(IntermediateDirectory)/src_ISDataMappings.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/ISDataMappings.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_ISDataMappings.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_ISDataMappings.cpp$(DependSuffix): ../../src/ISDataMappings.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_ISDataMappings.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_ISDataMappings.cpp$(DependSuffix) -MM "../../src/ISDataMappings.cpp"

$(IntermediateDirectory)/src_ISDataMappings.cpp$(PreprocessSuffix): ../../src/ISDataMappings.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_ISDataMappings.cpp$(PreprocessSuffix) "../../src/ISDataMappings.cpp"

$(IntermediateDirectory)/src_ISDisplay.cpp$(ObjectSuffix): ../../src/ISDisplay.cpp $(IntermediateDirectory)/src_ISDisplay.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/ISDisplay.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_ISDisplay.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_ISDisplay.cpp$(DependSuffix): ../../src/ISDisplay.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_ISDisplay.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_ISDisplay.cpp$(DependSuffix) -MM "../../src/ISDisplay.cpp"

$(IntermediateDirectory)/src_ISDisplay.cpp$(PreprocessSuffix): ../../src/ISDisplay.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_ISDisplay.cpp$(PreprocessSuffix) "../../src/ISDisplay.cpp"

$(IntermediateDirectory)/src_ISLogger.cpp$(ObjectSuffix): ../../src/ISLogger.cpp $(IntermediateDirectory)/src_ISLogger.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/ISLogger.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_ISLogger.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_ISLogger.cpp$(DependSuffix): ../../src/ISLogger.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_ISLogger.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_ISLogger.cpp$(DependSuffix) -MM "../../src/ISLogger.cpp"

$(IntermediateDirectory)/src_ISLogger.cpp$(PreprocessSuffix): ../../src/ISLogger.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_ISLogger.cpp$(PreprocessSuffix) "../../src/ISLogger.cpp"

$(IntermediateDirectory)/src_ISMatrix.c$(ObjectSuffix): ../../src/ISMatrix.c $(IntermediateDirectory)/src_ISMatrix.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/ISMatrix.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_ISMatrix.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_ISMatrix.c$(DependSuffix): ../../src/ISMatrix.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_ISMatrix.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_ISMatrix.c$(DependSuffix) -MM "../../src/ISMatrix.c"

$(IntermediateDirectory)/src_ISMatrix.c$(PreprocessSuffix): ../../src/ISMatrix.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_ISMatrix.c$(PreprocessSuffix) "../../src/ISMatrix.c"

$(IntermediateDirectory)/src_ISPose.c$(ObjectSuffix): ../../src/ISPose.c $(IntermediateDirectory)/src_ISPose.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/ISPose.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_ISPose.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_ISPose.c$(DependSuffix): ../../src/ISPose.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_ISPose.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_ISPose.c$(DependSuffix) -MM "../../src/ISPose.c"

$(IntermediateDirectory)/src_ISPose.c$(PreprocessSuffix): ../../src/ISPose.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_ISPose.c$(PreprocessSuffix) "../../src/ISPose.c"

$(IntermediateDirectory)/src_ISTcpClient.cpp$(ObjectSuffix): ../../src/ISTcpClient.cpp $(IntermediateDirectory)/src_ISTcpClient.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/ISTcpClient.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_ISTcpClient.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_ISTcpClient.cpp$(DependSuffix): ../../src/ISTcpClient.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_ISTcpClient.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_ISTcpClient.cpp$(DependSuffix) -MM "../../src/ISTcpClient.cpp"

$(IntermediateDirectory)/src_ISTcpClient.cpp$(PreprocessSuffix): ../../src/ISTcpClient.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_ISTcpClient.cpp$(PreprocessSuffix) "../../src/ISTcpClient.cpp"

$(IntermediateDirectory)/src_ISTcpServer.cpp$(ObjectSuffix): ../../src/ISTcpServer.cpp $(IntermediateDirectory)/src_ISTcpServer.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/ISTcpServer.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_ISTcpServer.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_ISTcpServer.cpp$(DependSuffix): ../../src/ISTcpServer.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_ISTcpServer.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_ISTcpServer.cpp$(DependSuffix) -MM "../../src/ISTcpServer.cpp"

$(IntermediateDirectory)/src_ISTcpServer.cpp$(PreprocessSuffix): ../../src/ISTcpServer.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_ISTcpServer.cpp$(PreprocessSuffix) "../../src/ISTcpServer.cpp"

$(IntermediateDirectory)/src_ISUtilities.cpp$(ObjectSuffix): ../../src/ISUtilities.cpp $(IntermediateDirectory)/src_ISUtilities.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/ISUtilities.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_ISUtilities.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_ISUtilities.cpp$(DependSuffix): ../../src/ISUtilities.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_ISUtilities.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_ISUtilities.cpp$(DependSuffix) -MM "../../src/ISUtilities.cpp"

$(IntermediateDirectory)/src_ISUtilities.cpp$(PreprocessSuffix): ../../src/ISUtilities.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_ISUtilities.cpp$(PreprocessSuffix) "../../src/ISUtilities.cpp"

$(IntermediateDirectory)/src_InertialSense.cpp$(ObjectSuffix): ../../src/InertialSense.cpp $(IntermediateDirectory)/src_InertialSense.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/InertialSense.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_InertialSense.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_InertialSense.cpp$(DependSuffix): ../../src/InertialSense.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_InertialSense.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_InertialSense.cpp$(DependSuffix) -MM "../../src/InertialSense.cpp"

$(IntermediateDirectory)/src_InertialSense.cpp$(PreprocessSuffix): ../../src/InertialSense.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_InertialSense.cpp$(PreprocessSuffix) "../../src/InertialSense.cpp"

$(IntermediateDirectory)/src_cltool.cpp$(ObjectSuffix): ../../src/cltool.cpp $(IntermediateDirectory)/src_cltool.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/cltool.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_cltool.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_cltool.cpp$(DependSuffix): ../../src/cltool.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_cltool.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_cltool.cpp$(DependSuffix) -MM "../../src/cltool.cpp"

$(IntermediateDirectory)/src_cltool.cpp$(PreprocessSuffix): ../../src/cltool.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_cltool.cpp$(PreprocessSuffix) "../../src/cltool.cpp"

$(IntermediateDirectory)/src_cltool_main.cpp$(ObjectSuffix): ../../src/cltool_main.cpp $(IntermediateDirectory)/src_cltool_main.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/cltool_main.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_cltool_main.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_cltool_main.cpp$(DependSuffix): ../../src/cltool_main.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_cltool_main.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_cltool_main.cpp$(DependSuffix) -MM "../../src/cltool_main.cpp"

$(IntermediateDirectory)/src_cltool_main.cpp$(PreprocessSuffix): ../../src/cltool_main.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_cltool_main.cpp$(PreprocessSuffix) "../../src/cltool_main.cpp"

$(IntermediateDirectory)/src_com_manager.c$(ObjectSuffix): ../../src/com_manager.c $(IntermediateDirectory)/src_com_manager.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/com_manager.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_com_manager.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_com_manager.c$(DependSuffix): ../../src/com_manager.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_com_manager.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_com_manager.c$(DependSuffix) -MM "../../src/com_manager.c"

$(IntermediateDirectory)/src_com_manager.c$(PreprocessSuffix): ../../src/com_manager.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_com_manager.c$(PreprocessSuffix) "../../src/com_manager.c"

$(IntermediateDirectory)/src_data_sets.c$(ObjectSuffix): ../../src/data_sets.c $(IntermediateDirectory)/src_data_sets.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/data_sets.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_data_sets.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_data_sets.c$(DependSuffix): ../../src/data_sets.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_data_sets.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_data_sets.c$(DependSuffix) -MM "../../src/data_sets.c"

$(IntermediateDirectory)/src_data_sets.c$(PreprocessSuffix): ../../src/data_sets.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_data_sets.c$(PreprocessSuffix) "../../src/data_sets.c"

$(IntermediateDirectory)/src_inertialSenseBootLoader.c$(ObjectSuffix): ../../src/inertialSenseBootLoader.c $(IntermediateDirectory)/src_inertialSenseBootLoader.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/inertialSenseBootLoader.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_inertialSenseBootLoader.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_inertialSenseBootLoader.c$(DependSuffix): ../../src/inertialSenseBootLoader.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_inertialSenseBootLoader.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_inertialSenseBootLoader.c$(DependSuffix) -MM "../../src/inertialSenseBootLoader.c"

$(IntermediateDirectory)/src_inertialSenseBootLoader.c$(PreprocessSuffix): ../../src/inertialSenseBootLoader.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_inertialSenseBootLoader.c$(PreprocessSuffix) "../../src/inertialSenseBootLoader.c"

$(IntermediateDirectory)/src_linked_list.c$(ObjectSuffix): ../../src/linked_list.c $(IntermediateDirectory)/src_linked_list.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/linked_list.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_linked_list.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_linked_list.c$(DependSuffix): ../../src/linked_list.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_linked_list.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_linked_list.c$(DependSuffix) -MM "../../src/linked_list.c"

$(IntermediateDirectory)/src_linked_list.c$(PreprocessSuffix): ../../src/linked_list.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_linked_list.c$(PreprocessSuffix) "../../src/linked_list.c"

$(IntermediateDirectory)/src_serialPort.c$(ObjectSuffix): ../../src/serialPort.c $(IntermediateDirectory)/src_serialPort.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/serialPort.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_serialPort.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_serialPort.c$(DependSuffix): ../../src/serialPort.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_serialPort.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_serialPort.c$(DependSuffix) -MM "../../src/serialPort.c"

$(IntermediateDirectory)/src_serialPort.c$(PreprocessSuffix): ../../src/serialPort.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_serialPort.c$(PreprocessSuffix) "../../src/serialPort.c"

$(IntermediateDirectory)/src_serialPortPlatform.c$(ObjectSuffix): ../../src/serialPortPlatform.c $(IntermediateDirectory)/src_serialPortPlatform.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/serialPortPlatform.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_serialPortPlatform.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_serialPortPlatform.c$(DependSuffix): ../../src/serialPortPlatform.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_serialPortPlatform.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_serialPortPlatform.c$(DependSuffix) -MM "../../src/serialPortPlatform.c"

$(IntermediateDirectory)/src_serialPortPlatform.c$(PreprocessSuffix): ../../src/serialPortPlatform.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_serialPortPlatform.c$(PreprocessSuffix) "../../src/serialPortPlatform.c"

$(IntermediateDirectory)/src_time_conversion.c$(ObjectSuffix): ../../src/time_conversion.c $(IntermediateDirectory)/src_time_conversion.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/time_conversion.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_time_conversion.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_time_conversion.c$(DependSuffix): ../../src/time_conversion.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_time_conversion.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_time_conversion.c$(DependSuffix) -MM "../../src/time_conversion.c"

$(IntermediateDirectory)/src_time_conversion.c$(PreprocessSuffix): ../../src/time_conversion.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_time_conversion.c$(PreprocessSuffix) "../../src/time_conversion.c"

$(IntermediateDirectory)/src_tinystr.cpp$(ObjectSuffix): ../../src/tinystr.cpp $(IntermediateDirectory)/src_tinystr.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/tinystr.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_tinystr.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_tinystr.cpp$(DependSuffix): ../../src/tinystr.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_tinystr.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_tinystr.cpp$(DependSuffix) -MM "../../src/tinystr.cpp"

$(IntermediateDirectory)/src_tinystr.cpp$(PreprocessSuffix): ../../src/tinystr.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_tinystr.cpp$(PreprocessSuffix) "../../src/tinystr.cpp"

$(IntermediateDirectory)/src_tinyxml.cpp$(ObjectSuffix): ../../src/tinyxml.cpp $(IntermediateDirectory)/src_tinyxml.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/tinyxml.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_tinyxml.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_tinyxml.cpp$(DependSuffix): ../../src/tinyxml.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_tinyxml.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_tinyxml.cpp$(DependSuffix) -MM "../../src/tinyxml.cpp"

$(IntermediateDirectory)/src_tinyxml.cpp$(PreprocessSuffix): ../../src/tinyxml.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_tinyxml.cpp$(PreprocessSuffix) "../../src/tinyxml.cpp"

$(IntermediateDirectory)/src_tinyxmlerror.cpp$(ObjectSuffix): ../../src/tinyxmlerror.cpp $(IntermediateDirectory)/src_tinyxmlerror.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/tinyxmlerror.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_tinyxmlerror.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_tinyxmlerror.cpp$(DependSuffix): ../../src/tinyxmlerror.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_tinyxmlerror.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_tinyxmlerror.cpp$(DependSuffix) -MM "../../src/tinyxmlerror.cpp"

$(IntermediateDirectory)/src_tinyxmlerror.cpp$(PreprocessSuffix): ../../src/tinyxmlerror.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_tinyxmlerror.cpp$(PreprocessSuffix) "../../src/tinyxmlerror.cpp"

$(IntermediateDirectory)/src_tinyxmlparser.cpp$(ObjectSuffix): ../../src/tinyxmlparser.cpp $(IntermediateDirectory)/src_tinyxmlparser.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/tinyxmlparser.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_tinyxmlparser.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_tinyxmlparser.cpp$(DependSuffix): ../../src/tinyxmlparser.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_tinyxmlparser.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_tinyxmlparser.cpp$(DependSuffix) -MM "../../src/tinyxmlparser.cpp"

$(IntermediateDirectory)/src_tinyxmlparser.cpp$(PreprocessSuffix): ../../src/tinyxmlparser.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_tinyxmlparser.cpp$(PreprocessSuffix) "../../src/tinyxmlparser.cpp"

$(IntermediateDirectory)/src_ISSerialPort.cpp$(ObjectSuffix): ../../src/ISSerialPort.cpp $(IntermediateDirectory)/src_ISSerialPort.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/ISSerialPort.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_ISSerialPort.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_ISSerialPort.cpp$(DependSuffix): ../../src/ISSerialPort.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_ISSerialPort.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_ISSerialPort.cpp$(DependSuffix) -MM "../../src/ISSerialPort.cpp"

$(IntermediateDirectory)/src_ISSerialPort.cpp$(PreprocessSuffix): ../../src/ISSerialPort.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_ISSerialPort.cpp$(PreprocessSuffix) "../../src/ISSerialPort.cpp"

$(IntermediateDirectory)/src_ISStream.cpp$(ObjectSuffix): ../../src/ISStream.cpp $(IntermediateDirectory)/src_ISStream.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/ISStream.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_ISStream.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_ISStream.cpp$(DependSuffix): ../../src/ISStream.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_ISStream.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_ISStream.cpp$(DependSuffix) -MM "../../src/ISStream.cpp"

$(IntermediateDirectory)/src_ISStream.cpp$(PreprocessSuffix): ../../src/ISStream.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_ISStream.cpp$(PreprocessSuffix) "../../src/ISStream.cpp"

$(IntermediateDirectory)/src_ISEarth.c$(ObjectSuffix): ../../src/ISEarth.c $(IntermediateDirectory)/src_ISEarth.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/pi/share/IS-src/cpp/SDK/src/ISEarth.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_ISEarth.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_ISEarth.c$(DependSuffix): ../../src/ISEarth.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_ISEarth.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_ISEarth.c$(DependSuffix) -MM "../../src/ISEarth.c"

$(IntermediateDirectory)/src_ISEarth.c$(PreprocessSuffix): ../../src/ISEarth.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_ISEarth.c$(PreprocessSuffix) "../../src/ISEarth.c"


-include $(IntermediateDirectory)/*$(DependSuffix)
##
## Clean
##
clean:
	$(RM) ./Debug/*$(ObjectSuffix)
	$(RM) ./Debug/*$(DependSuffix)
	$(RM) $(OutputFile)
	$(RM) ".build-debug/CLTool"


