
/* -----------------------------------------------------------
   Reads the .blocks file to get the blocks information
   
   creates extern vars: 
            numBlocks, maxBlocksDegree, xBlocksCord, yBlocksCord;
----------------------------------------------------------- */
void readBlocksFile(char benchmarkPath[], char blocksFile[])
{
    FILE *fp;
    char file_name[BUFFERSIZE], temp[BUFFERSIZE], nodeName[BUFFERSIZE], pinDirection[5];
    char line[LINESIZE];
    long blockIndex;
    int degree, prevElements, netNo, k, startPointer;
    float xOffset, yOffset;
    char * pch;
    
    
    strcpy(file_name, benchmarkPath);
    strcat(file_name, "/");
    strcat(file_name, netsFile);
    
    if((fp=fopen(file_name, "r"))==NULL) {
        sprintf(error_text, "bookshelf_IO: Cannot open %s file", file_name);
        runtimeError(error_text);
    }
    
    numBlocks = 0;
    while(!feof(fp)) {
        fgets(line, LINESIZE, fp);
        sscanf(line, "%s\t%*s\n", temp);
        
        if(strlen(line) < 5 || temp[0] == '#' || strcmp(temp, "UCLA") == 0)
            continue;
        
        if(strcmp(temp, "NumBlocks") == 0) {
            sscanf(line, "NumBlocks\t:\t%d\n", &numBlocks);
        } else if(strcmp(temp, "MaxBlocksDegree") == 0) {
            sscanf(line, "MaxBlocksDegree\t:\t%d\n", &maxBlocksDegree);
            break;
        } else {}
    }
    fclose(fp);
    
    if(numBlocks) {
        sprintf(error_text, "bookshelf_IO: NumNets = %d,     NumPins = %d ", numNets, numPins);
        runtimeError(error_text);
    }
    
    if(xBlockCoord == NULL && yBlockCoord == NULL) {
//        xBlockCoord  = vector(1,numBlocks+1);
//        yBlockCoord  = vector(1,numBlocks+1);
        xBlockCord = matrix (1,numBlocks,1,maxBlocksDegree);
        yBlockCord = matrix (1,numBlocks,1,maxBlocksDegree);
        
    } else {
        runtimeError("bookshelf_IO: allocation error in readNetsFile()");
    }
    
    if((fp=fopen(file_name, "r"))==NULL) {
        sprintf(error_text, "bookshelf_IO: Cannot open %s file", file_name);
        runtimeError(error_text);
    }
//    printf("Reading %s ...\n", netsFile);
    
    while(!feof(fp)) {
        *line = '\0';
        fgets(line, LINESIZE, fp);
        sscanf(line, "%s\t%*s\n", temp);
        
        if(strlen(line) < 5 || temp[0] == '#' || strcmp(temp, "UCLA") == 0)
            continue;
        
        if(strcmp(temp, "NumBlocks") == 0 || strcmp(temp, "maxBlocksDegree") == 0)
            continue;
        
        if(strcmp(temp, "NetDegree") == 0) {
            netNo++;
            *temp = '\0';
            sscanf(line, "NetDegree\t:\t%d\t%s\n", &degree, temp);
            if(strcmp(temp, "") == 0)
                sprintf(temp, "net_%d", netNo);
            netName[netNo] = (char *) malloc((strlen(temp)+1)*sizeof(char));
            strcpy(netName[netNo], temp);
            
            netlistIndex[netNo] = netlistIndex[netNo-1] + prevElements;
            startPointer = netlistIndex[netNo];
            prevElements = degree;
            k = 1;
        } else {
            pch = strtok(line, " ");
            while (pch!=NULL){
                
                pch = strtok(line, " ");
            }
            xOffset = yOffset = 0.0;
            *pinDirection = '\0';
            
            sscanf(line, "%s%s", nodeName, pinDirection);
            if(pinDirection[0] == ':')
                sscanf(line, "%*s%*s%f%f", &xOffset, &yOffset);
            else
                sscanf(line, "%s%s%*s%f%f", nodeName, pinDirection, &xOffset, &yOffset);
            
            if(strcmp(pinDirection, "") == 0 || pinDirection[0] == ':')
                strcpy(pinDirection, "B");
            
            nodeIndex = getIndex(nodeName);
            
            netlist[startPointer+k-1]    = nodeIndex;
            xPinOffset[startPointer+k-1] = xOffset;
            yPinOffset[startPointer+k-1] = yOffset;
            k++;
        }
    }
    netlistIndex[numNets+1] = netlistIndex[numNets] + prevElements;
    netlist[netlistIndex[numNets+1]] = 0;
    
    fclose(fp); 
    if(netNo != numNets) {
        sprintf(error_text, "bookshelf_IO: NumNets (%d) != Number of Net descriptions (%d)", 
                numNets, netNo);
        runtimeError(error_text);
    }
    
    if(netlistIndex[numNets+1]-1 != numPins) {
        sprintf(error_text, "bookshelf_IO: NumPins (%d) != Number of Pin descriptions (%d)", 
                numPins, netlistIndex[numNets+1]-1);
        runtimeError(error_text);
    }
    
#if(DEBUG)
int i, j;
for(i=1; i<=numNets; i++) {
    printf("**%d**  ", netlistIndex[i+1]-netlistIndex[i]);
    for(j=netlistIndex[i]; j<netlistIndex[i+1]; j++) {
        printf("(%d) %.2f %.2f  ", netlist[j], xPinOffset[j], yPinOffset[j]);
    }
    printf("\n");    
}
#endif
}
