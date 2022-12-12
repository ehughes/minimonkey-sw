# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import os

StartIndex = 0
StopIndex = 65

BaseFileNameString = 'Sloan-240x320-'
FileExtention = '.jpg'

ImageOutputFilename = 'anim_';
ImageOutputC_BaseName = 'anim_';
ImageOutputFolder = 'output'

SectionDecorator = '';

if not os.path.exists(ImageOutputFolder):
    os.makedirs(ImageOutputFolder);


IndexOutputFileC_Name = ImageOutputFilename+'index.c'
IndexOutputFileH_Name = ImageOutputFilename+'index.h'

IndexOutputFile = open(ImageOutputFolder + '/' + IndexOutputFileC_Name,'w+')
IndexH_OutputFile = open(ImageOutputFolder + '/' + IndexOutputFileH_Name,'w+')

IndexH_OutputFile.write("#include <stdint.h>\n\n");
IndexH_OutputFile.write("#ifndef __" + ImageOutputC_BaseName + "INDEX_H\n");
IndexH_OutputFile.write("#define __" + ImageOutputC_BaseName + "INDEX_H\n\n");


IndexH_OutputFile.write("typedef struct {\n");
IndexH_OutputFile.write("\n");
IndexH_OutputFile.write("	const uint8_t * JPEG_Data;\n");
IndexH_OutputFile.write("	const uint32_t JPEG_DataSize;\n");
IndexH_OutputFile.write("\n");
IndexH_OutputFile.write("}JPEG_Record;\n\n");

IndexH_OutputFile.write("#define NUM_JPEGS  " + str(StopIndex-StartIndex) + " \n");
IndexH_OutputFile.write("\nextern JPEG_Record MyJPEGs[NUM_JPEGS]; \n\n");


IndexOutputFile.write("#include \"" + IndexOutputFileH_Name + "\"\n\n");

IndexOutputFile.write("JPEG_Record MyJPEGs[NUM_JPEGS] ={ \n");


for i in range(StartIndex,StopIndex):
    
    InputFileName = BaseFileNameString + str(i) + FileExtention;
    print('Processing ' +InputFileName)
    
    
    file = open(InputFileName, "rb");
        
    BinData = file.read();

    BinDataLength = len(BinData)
    BinVariableName = ImageOutputC_BaseName + str(i);

    ImageOutputFileC_Name = ImageOutputFilename+str(i)+'.c'
    ImageOutputFileH_Name = ImageOutputFilename+str(i)+'.h'


    ImageOutputFile = open(ImageOutputFolder + '/' + ImageOutputFileC_Name,'w+')
    ImageH_OutputFile = open(ImageOutputFolder + '/' + ImageOutputFileH_Name,'w+')

 
    ImageH_OutputFile.write("#include <stdint.h>\n\n");
    ImageH_OutputFile.write("#ifndef __" + BinVariableName+"_H\n");
    ImageH_OutputFile.write("#define __" + BinVariableName+"_H\n\n");
    
    ImgSizeMacro  = BinVariableName + "__size";
    
    ImageH_OutputFile.write("#define " + ImgSizeMacro+ " " +str(BinDataLength) + "  \n");
    ImageH_OutputFile.write("extern const uint8_t " + BinVariableName + "[" + str(BinDataLength)+"];  \n\n");
    ImageH_OutputFile.write("#endif \n");
    
     
    IndexOutputFile.write("{"+BinVariableName+"," + ImgSizeMacro + "}");
    
    if(i <StopIndex ):
        IndexOutputFile.write(",\n");
    else:
        IndexOutputFile.write("\n");
    
    ImgInclude = "#include \"" + ImageOutputFileH_Name + "\"\n";
    ImageOutputFile.write(ImgInclude);
    IndexH_OutputFile.write(ImgInclude);
             
    ImageOutputFile.write("\n" + SectionDecorator + "const uint8_t " + BinVariableName + "["+str(BinDataLength)+"] = { \n");
       
    Index = 0                 
    for Data in BinData:
        ImageOutputFile.write(str(Data))
        if(Index%32 == 31):
            ImageOutputFile.write('\n');
        Index = Index + 1;
    
        if(Index!=BinDataLength):
            ImageOutputFile.write(',');
            
        
    ImageOutputFile.write("};\n");
                          
    ImageH_OutputFile.close();
    
    ImageOutputFile.close();
    file.close();

IndexH_OutputFile.write("\n#endif \n");

IndexOutputFile.write("}; \n");


IndexOutputFile.close();
IndexH_OutputFile.close();