import SubFunctionBlocks as FBlocksAPI 
import subMAVFunctionSettings as MAVFSettingsAPI
import sys,os
import StructDataGen
import struct, array

class dataType():
	def __init__( self, typeName, baseTypeName, postQualifier = ''):
		self.typeName = typeName
		self.baseTypeName = baseTypeName
		self.postQualifier = postQualifier

class CFiles():
	def __init__( self ):
		self.componentName = "UNKNOWN"
	
		self.dataTypes = []
		self.dataTypes.append(dataType('Register',  'uint16_t'))	#, ' \t: 8'
		self.dataTypes.append(dataType('Percent',    'fractional'))
		self.dataTypes.append(dataType('Fraction',   'fractional'))
		self.dataTypes.append(dataType('int14',      'fractional'))
		self.dataTypes.append(dataType('int16',      'int16_t'))


	def m_findRegisterIndexWithName ( self, regName ):
		index = 0
		for item in self.FBSettings.registers.register:
			if regName == item.identifier:
				return index
			index = index + 1
		return 0


	def m_findTypeIndexWithName ( self, typeName ):
		index = 0
		for FBlock in self.FBlocks:
			if typeName == FBlock.header.name:
				return index
			index = index + 1
		return 0

	def m_getSetValueFromType ( self, setType  ):
		if setType == 'SET':
			return 0
		
		if setType == 'ADD':
			return 1
	
		if setType == 'CLEAR':
			return 2
		
		return 3

	def getBaseType ( self, fullType):
		for dType in self.dataTypes:
			if fullType == dType.typeName:
				return dType
		return "failed to find base type, you need to edit the source code"

	def writeFiles( self, filePath, componentName , FBSettings, FBlocks):
		self.componentName = componentName
		self.FBSettings = FBSettings
		self.FBlocks = FBlocks
		self.filePath = filePath
		
		self.DataGen = StructDataGen.structDataGen(self.FBSettings, self.FBlocks)
		
		self.writeRegisterHeader()
#		self.writeTypesHeader()
#		self.writeFunctionsSetup()

		self.writeTypesHeaderCompact()
		self.writeFunctionsSetupCompact()
		self.writeTypesTable()

		self.writeStringTables()

 
 # String table header
	def writeStringTables( self ):
		self.StrTableFileName = "flexiFunctionStrTable.c"
		headerFile = open(os.path.join(self.filePath, self.StrTableFileName), "w")

		headerFile.write("// pyFEdit generated file - DO NOT EDIT\n\n\n")
		 
		headerFile.write('#include "flexiFunctionTypes.h"\n\n')
	
		headerFile.write("const ff_string ff_register_strings[]\n{\n")
	
		for reg in self.FBSettings.registers.register:
			headerFile.write('"')
			headerFile.write(reg.identifier)
			headerFile.write('",\n')
 
		headerFile.write('" "\n')		# Last entry is written with a space as first character
	
		headerFile.write("\n};\n\n")
	
			
		headerFile.write("#endif\n")
		
		headerFile.close()
		
	#Registers enumeration header 
	def writeRegisterHeader( self ):
		self.RegHeaderFileName = "flexiFunctionRegisters.h"
		headerFile = open(os.path.join(self.filePath, self.RegHeaderFileName), "w")
	
		headerFile.write("#ifndef FLEXIFUNCTION_REGISTERS_H\n")
		headerFile.write("#define FLEXIFUNCTION_REGISTERS_H\n")

		headerFile.write("// pyFEdit generated file - DO NOT EDIT\n\n\n")
	
		#=======================================================================
		# headerFile.write("typedef enum\n{\n")
		# for reg in self.FBSettings.registers.register:
		#	headerFile.write("REG_")
		#	headerFile.write(reg.identifier)
		#	headerFile.write(",\n")	
		# headerFile.write("REG_MAX")
		# headerFile.write("\n};\n\n")
		#=======================================================================
		
		headerFile.write("typedef enum\n{\n")
		for input in self.FBSettings.inputRegs.input:
			headerFile.write("VIRTUAL_INPUT_")
			headerFile.write(input.register)
			headerFile.write(",\n")	
		headerFile.write("};\n\n")

		headerFile.write("typedef enum\n{\n")
		for output in self.FBSettings.outputRegs.output:
			headerFile.write("VIRTUAL_OUTPUT_")
			headerFile.write(output.register)
			headerFile.write(",\n")	
		headerFile.write("};\n\n")
			
		headerFile.write("#endif\n")
		
		headerFile.close()

# Flexifunction types header
	def writeTypesHeader( self ):
		self.TypesHeaderFileName = "flexiFunctionTypes.h"
		headerFile = open(os.path.join(self.filePath, self.TypesHeaderFileName), "w")
	
		self.pFunctionPointerName = 'pflexFunction'
	
		headerFile.write("#ifndef FLEXIFUNCTION_TYPES_H\n")
		headerFile.write("#define FLEXIFUNCTION_TYPES_H\n\n")
	
		headerFile.write('#include <dsp.h>\n\n')
	
		headerFile.write('#include "flexifunction_options.h"\n\n\n')
	
		headerFile.write('/****************************************************************/\n')
		headerFile.write("// pyFEdit generated file - DO NOT EDIT\n\n\n")
	
	
		self.funcNames = []
		self.structNames = []
	
		for func in self.FBlocks:
			funcName = func.header.name.lower() + '_function'
			self.funcNames.append(funcName)
	
			headerFile.write("typedef struct tagFuncData_" + func.header.name + "\n")
			headerFile.write("{\n")
			
			for param in func.setting:
				paramType = self.getBaseType(param.type_)
				paramName = param.name
				headerFile.write("\t" + paramType.baseTypeName + "\t" + paramName + paramType.postQualifier + ";\n")
				
			structName = func.header.name
			structName = structName.lower()
			self.structNames.append(structName)
			headerFile.write("} FuncData_" + structName +";\n\n\n")
	
		headerFile.write("typedef union\n")
		headerFile.write("{\n")
		for sName in self.structNames:
			headerFile.write("\tFuncData_" + sName + "\t" + sName + ";\n")
		headerFile.write("} functionData;\n\n\n")
	
		headerFile.write('/****************************************************************/\n')
		headerFile.write('// Function Settings\n\n\n');
	
		headerFile.write('typedef struct tagFunctionSettings\n')
		headerFile.write('{\n')
		headerFile.write('  uint16_t    functionType        : 6 ;   // The type of function\n')
		headerFile.write('  uint16_t    setValue            : 2 ;   // The destination is set(0) added(1) or cleared (2)\n')
		headerFile.write('  uint16_t    dest                : 8 ;   // The destination register\n')
		headerFile.write('  functionData    data;\n')
		headerFile.write('} functionSetting;\n\n\n')
	
	
		headerFile.write('/****************************************************************/\n')
		headerFile.write('// Mixer functions\n\n\n')
		headerFile.write('typedef int16_t (*pflexFunction)(functionSetting*, fractional*); \n\n\n')
	
		for fName in self.funcNames:
			headerFile.write('extern fractional ' + fName + '(functionSetting* pSetting, fractional* pRegisters);\n')
	
		headerFile.write('\n\n\n')
	
#		headerFile.write('extern pflexFunction flexiFunctions [];\n\n\n')
# The flexiFunctions need to be declared by individual components, not the 
	
		headerFile.write('extern void runFlexiFunctions( functionSetting* pSettings, fractional* pRegisters, unsigned int max_functions );\n\n')

		headerFile.write('extern const pflexFunction flexiFunctions [];\n\n')
		
		headerFile.write('typedef struct tagFLEXIFUNCTION_DATASET\n')
		headerFile.write('{\n')
		headerFile.write('	uint8_t inputs_directory[FLEXIFUNCTION_MAX_DIRECTORY_SIZE];\n')
		headerFile.write('	uint8_t outputs_directory[FLEXIFUNCTION_MAX_DIRECTORY_SIZE];\n')
		headerFile.write('	functionSetting flexiFunction_data[FLEXIFUNCTION_MAX_FUNCS];\n')
		headerFile.write('	uint16_t flexiFunctionsUsed;\n')
		headerFile.write('} FLEXIFUNCTION_DATASET;\n\n')

		headerFile.write('extern FLEXIFUNCTION_DATASET flexiFunction_dataset;\n\n')		

		self.registersStructName = 'flexiFunction_registers'
		headerFile.write('extern fractional flexiFunction_registers[FLEXIFUNCTION_MAX_REGS];\n\n');

		headerFile.write('extern unsigned char get_input_register_index_from_directory(unsigned char virtual_index);\n')
		headerFile.write('extern unsigned char get_output_register_index_from_directory(unsigned char virtual_index);\n\n\n')
#		headerFile.write('extern unsigned char inputs_directory_size;\n\n\n')
#		headerFile.write('extern unsigned char outputs_directory_size;\n\n\n')
			
		headerFile.write("#endif\n")
		
		headerFile.close()

# Flexifunction types table implementation
	def writeTypesTable( self ):
		self.TypesTableFileName = "flexiFunctionTypes.c"
		headerFile = open(os.path.join(self.filePath, self.TypesTableFileName), "w")

		headerFile.write('#include "flexiFunctionTypes.h"\n\n')

		headerFile.write('/****************************************************************/\n')
		headerFile.write("//\tpyFEdit generated file - DO NOT EDIT\n\n\n")
	
		self.pFunctionTableName = 'flexiFunctions'
	
		headerFile.write('const pflexFunction flexiFunctions[] =\n')

		headerFile.write('\t{\n')
	
		for funcName in self.funcNames:
			headerFile.write('    &' + funcName + ',\n')
	#            headerFile.write()
		headerFile.write('\t};\n\n\n')

		headerFile.write('const unsigned char functionParamSizes[] ={')
		
		for func in self.FBlocks:
			paramsSize = 0
			for param in func.setting:
				paramType = self.getBaseType(param.type_)
				paramsSize = paramsSize + 2
			headerFile.write(str(paramsSize) + ', ')
		
		headerFile.write('};\n\n\n')
	
		headerFile.close()

#===============================================================================
# # Flexifunction settings
#	def writeFunctionsSetup( self ):
#		print("write functions setup file")
#		self.FunctionSetupFileName = "flexiFunctionSettings.c"
#		setupFile = open(os.path.join(self.filePath,self.FunctionSetupFileName), "w")
#	
#		setupFile.write('#include "flexiFunctionTypes.h"\n\n')
#	
#		setupFile.write('#define RMAX15 0b0110000000000000  //  1.5 in 2.14 format\n')
#		setupFile.write('#define RMAX   0b0100000000000000  //  1.0 in 2.14 format\n\n\n')
#			
#		setupFile.write('#define PercenttoQ14(n) ((int16_t)(n * 163.84))\n')
#		setupFile.write('#define FloattoQ14(n) ((int16_t)(n * 16384))\n')
#		setupFile.write('\n\n\n')
#		
# #		setupFile.write('functionSetting flexifunction_buffer[80] = {};\n\n\n')
#		
#		setupFile.write('FLEXIFUNCTION_DATASET flexiFunction_dataset =\n')
# #		setupFile.write('functionSetting flexiFunction_data [FLEXIFUNCTION_MAX_FUNCS] = \n')
#		setupFile.write('{\n')
#		
#		setupFile.write('\t{')
#		for inputReg in self.FBSettings.inputRegs.input:
#			regIndex = self.m_findRegisterIndexWithName(inputReg.get_register())
#			setupFile.write(str(regIndex) + ', ')
#		setupFile.write('},\n')		
#		
#		setupFile.write('\t{')
#		for outputReg in self.FBSettings.outputRegs.output:
#			regIndex = self.m_findRegisterIndexWithName(outputReg.get_register())
#			setupFile.write(str(regIndex) + ', ')
#		setupFile.write('},\n')		
#		
#		setupFile.write('\t{\n')
# 
#		for func in self.FBSettings.functions.function:
#				
#			typeIndex   =   self.m_findTypeIndexWithName(func.header.functionType)
#			destIndex    =   self.m_findRegisterIndexWithName(func.header.destReg)
#			actionVal   =   self.m_getSetValueFromType(func.header.action)
#			setupFile.write('\t\t{'+ '{:d}, {:d}, {:d}, '.format(typeIndex, actionVal, destIndex) + '\t{.')
#	
#			setupFile.write(self.structNames[typeIndex] + ' = { ')
#			
#			paramIndex = 0
#			for param in func.setting:
#				paramType = self.FBlocks[typeIndex].setting[paramIndex].type_
#				if paramIndex > 0:
#					setupFile.write(',')
#				if paramType == 'Fraction':
#					setupFile.write('FloattoQ14(' + param.value + ')')
#				if paramType == 'Percent':
#					setupFile.write('PercenttoQ14(' + param.value + ')')
#				if paramType == 'Register':
#					regIndex = self.m_findRegisterIndexWithName(param.value)
#					print('Register ' + param.value + ' index {:d}'.format(regIndex) )
#					setupFile.write( '{:d}'.format(regIndex) )
#				if paramType == 'int16':
#					setupFile.write(param.value)
#				if paramType == 'int14':
#					setupFile.write(param.value)
#				paramIndex = paramIndex + 1
#			setupFile.write('} } },\n')
#		setupFile.write('\t},\n')
# 
#		setupFile.write('\t{ ' + '{:d}'.format(len(self.FBSettings.functions.function)) + '},\n')
#		setupFile.write('};\n\n\n')
#===============================================================================


# Flexifunction types header
	def writeTypesHeaderCompact( self ):
		headerFile = open(os.path.join(self.filePath, "flexiFunctionTypes.h"), "w")
	
		self.pFunctionPointerName = 'pflexFunction'
	
		headerFile.write("#ifndef FLEXIFUNCTION_TYPES_COMPACT_H\n")
		headerFile.write("#define FLEXIFUNCTION_TYPES_COMPACT_H\n\n")
	
		headerFile.write('#include <dsp.h>\n')
		headerFile.write('#include <stdint.h>\n\n')
	
		headerFile.write('#include "flexifunction_options.h"\n\n\n')
	
		headerFile.write('/****************************************************************/\n')
		headerFile.write("// pyFEdit generated file - DO NOT EDIT\n\n\n")
	
	
		self.funcNames = []
		self.structNames = []
	
		for func in self.FBlocks:
			funcName = func.header.name.lower() + '_function'
			self.funcNames.append(funcName)
	
			headerFile.write("typedef struct tagFuncData_" + func.header.name + "\n")
			headerFile.write("{\n")
			
			for param in func.setting:
				paramType = self.getBaseType(param.type_)
				paramName = param.name
				headerFile.write("\t" + paramType.baseTypeName + "\t" + paramName + paramType.postQualifier + ";\n")
				
			structName = func.header.name
			structName = structName.lower()
			self.structNames.append(structName)
			headerFile.write("} FuncData_" + structName +";\n\n\n")
	
		headerFile.write("typedef union\n")
		headerFile.write("{\n")
		for sName in self.structNames:
			headerFile.write("\tFuncData_" + sName + "\t" + sName + ";\n")
		headerFile.write("} functionData;\n\n\n")
	
		headerFile.write('/****************************************************************/\n')
		headerFile.write('// Function Settings\n\n\n');
	
		headerFile.write('typedef struct tagFunctionSettings\n')
		headerFile.write('{\n')
		headerFile.write('  uint16_t    functionType        : 6 ;   // The type of function\n')
		headerFile.write('  uint16_t    setValue            : 2 ;   // The destination is set(0) added(1) or cleared (2)\n')
		headerFile.write('  uint16_t    dest                : 8 ;   // The destination register\n')
		headerFile.write('  functionData    data;\n')
		headerFile.write('} functionSetting;\n\n\n')
	
	
		headerFile.write('/****************************************************************/\n')
		headerFile.write('// Mixer functions\n\n\n')
		headerFile.write('typedef int16_t (*pflexFunction)(functionSetting*, fractional*); \n\n\n')
	
		for fName in self.funcNames:
			headerFile.write('extern fractional ' + fName + '(functionSetting* pSetting, fractional* pRegisters);\n')
	
		headerFile.write('\n\n\n')
	
		headerFile.write('extern void runFlexiFunctions( void );\n\n')

		headerFile.write('extern const pflexFunction flexiFunctions [];\n\n')
		
		headerFile.write('typedef struct tagFLEXIFUNCTION_DATASET\n')
		headerFile.write('{\n')
		headerFile.write('	uint8_t inputs_directory[FLEXIFUNCTION_MAX_DIRECTORY_SIZE];\n')
		headerFile.write('	uint8_t outputs_directory[FLEXIFUNCTION_MAX_DIRECTORY_SIZE];\n')
		headerFile.write('	uint16_t flexiFunction_directory[FLEXIFUNCTION_MAX_FUNCS];\n')
		headerFile.write('	uint8_t flexiFunction_data[FLEXIFUNCTION_MAX_DATA_SIZE];\n')
		headerFile.write('	uint16_t flexiFunctionsUsed;\n')
		headerFile.write('} FLEXIFUNCTION_DATASET;\n\n')

		headerFile.write('extern FLEXIFUNCTION_DATASET flexiFunction_dataset;\n\n')		

		self.registersStructName = 'flexiFunction_registers'
		headerFile.write('extern fractional flexiFunction_registers[FLEXIFUNCTION_MAX_REGS];\n\n');

		headerFile.write('extern unsigned char get_input_register_index_from_directory(unsigned char virtual_index);\n')
		headerFile.write('extern unsigned char get_output_register_index_from_directory(unsigned char virtual_index);\n\n\n')
			
		headerFile.write('typedef char ff_string[32];\n\n')
		headerFile.write('extern const ff_string ff_function_strings[];\n\n')
		headerFile.write('extern const ff_string ff_register_strings[];\n\n')
			
		headerFile.write("#endif\n")
		
		headerFile.close()


		
	def writeFunctionsSetupCompact( self ):
		print("write functions setup file")
		self.FunctionSetupFileName = "flexiFunctionSettings.c"
		setupFile = open(os.path.join(self.filePath,self.FunctionSetupFileName), "w")
	
		setupFile.write('#include "flexiFunctionTypes.h"\n\n')
		
		setupFile.write('FLEXIFUNCTION_DATASET flexiFunction_dataset =\n')
#		setupFile.write('functionSetting flexiFunction_data [FLEXIFUNCTION_MAX_FUNCS] = \n')
		setupFile.write('{\n')
		
		setupFile.write('\t{')
		for inputReg in self.FBSettings.inputRegs.input:
			regIndex = self.m_findRegisterIndexWithName(inputReg.get_register())
			setupFile.write(str(regIndex) + ', ')
		setupFile.write('},\n')		
		
		setupFile.write('\t{')
		for outputReg in self.FBSettings.outputRegs.output:
			regIndex = self.m_findRegisterIndexWithName(outputReg.get_register())
			setupFile.write(str(regIndex) + ', ')
		setupFile.write('},\n')		
		

		functionDatas = []
		index = 0
		funcAddress = 0
		
		# Generate functions directory and functions data 
		for func in self.FBSettings.functions.function:
			func_settings =  self.DataGen.m_FunctionGenerateStruct(index)
			func_settings.funcAddress = funcAddress
			functionDatas.append(func_settings)
			index += 1
			funcAddress += func_settings.funcSize

		setupFile.write('\t{')
		for func in functionDatas:
			setupFile.write('0x{:X}'.format(func.funcAddress) + ', ')
		setupFile.write('},\n')	

		setupFile.write('\t{\n\t')
		for func in functionDatas:
			for byteIndex in range(0, func.funcSize):
				strval = struct.unpack('B', func.funcData[byteIndex])
				[val] = strval
				setupFile.write('0x{:X}, '.format(val))
			setupFile.write('\n\t')
		setupFile.write('},\n')	


		setupFile.write('\t{ ' + '{:d}'.format(len(self.FBSettings.functions.function)) + '},\n')
		setupFile.write('};\n\n\n')


		setupFile.close()

	
	

	
