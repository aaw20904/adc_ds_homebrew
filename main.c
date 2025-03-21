#include <stdio.h>
#include <stdlib.h>
/*
NOTE: This file was compiled by the Embarcadero DEV C++ IDE. 
I use for filtering only two functions:     filter128_v3(*auxPtrToBitStream, auxPtrToFilteredBuffer);   IIR	 for bitstream filtering (Fcut=Fbitstream/ 128, when OSR=64)
                                            float DigFil3K_1(invar) ;    IIR post-filtering    (Fcut=Fbitstream/ 128, when OSR=64)
                                            float DigFil(invar);     FIR high-pass filter (DC removing)
Other filter functions is experimental - for example CIC filters and not used in main procedure
                                            
*/
float DigFil3K_1(invar)
float invar;
/******************************************************************************/
/* 199 Tap Low Pass Hanning                                                   */
/* Finite Impulse Response                                                    */
/* Sample Frequency = 432.0 KHz                                               */                                                           */
/* Arithmetic Precision = 4 Digits                                            */
/*                                                                            */
/* Pass Band Frequency = 3.000 KHz                                            */
/*                                                                            */
/******************************************************************************/                                         */
/******************************************************************************/

{
    float sumnum=0.0; int i=0;
    static float states[198] = {
        0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
        0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0
    };
    static float znum[199] = {
        0.0,-7.782e-07,-3.077e-06,-6.827e-06,-1.194e-05,-1.83e-05,-2.577e-05,-3.419e-05,-4.34e-05,-5.319e-05,
        -6.335e-05,-7.363e-05,-8.377e-05,-9.352e-05,-1.026e-04,-1.106e-04,-1.173e-04,-1.223e-04,-1.254e-04,-1.26e-04,
        -1.239e-04,-1.187e-04,-1.099e-04,-9.728e-05,-8.036e-05,-5.877e-05,-3.211e-05,0.0,3.794e-05,8.207e-05,
        1.328e-04,1.904e-04,2.553e-04,3.277e-04,4.081e-04,4.966e-04,5.935e-04,6.991e-04,8.137e-04,9.373e-04,
        1.07e-03,1.212e-03,1.364e-03,1.526e-03,1.697e-03,1.877e-03,2.067e-03,2.267e-03,2.476e-03,2.695e-03,
        2.922e-03,3.159e-03,3.404e-03,3.658e-03,3.92e-03,4.189e-03,4.466e-03,4.75e-03,5.04e-03,5.337e-03,
        5.639e-03,5.945e-03,6.257e-03,6.571e-03,6.889e-03,7.21e-03,7.532e-03,7.856e-03,8.179e-03,8.503e-03,
        8.825e-03,9.146e-03,9.464e-03,9.779e-03,1.009e-02,1.04e-02,1.07e-02,1.099e-02,1.128e-02,1.156e-02,
        1.183e-02,1.209e-02,1.234e-02,1.258e-02,1.281e-02,1.303e-02,1.324e-02,1.343e-02,1.361e-02,1.377e-02,
        1.393e-02,1.406e-02,1.418e-02,1.429e-02,1.438e-02,1.445e-02,1.451e-02,1.455e-02,1.458e-02,1.459e-02,
        1.458e-02,1.455e-02,1.451e-02,1.445e-02,1.438e-02,1.429e-02,1.418e-02,1.406e-02,1.393e-02,1.377e-02,
        1.361e-02,1.343e-02,1.324e-02,1.303e-02,1.281e-02,1.258e-02,1.234e-02,1.209e-02,1.183e-02,1.156e-02,
        1.128e-02,1.099e-02,1.07e-02,1.04e-02,1.009e-02,9.779e-03,9.464e-03,9.146e-03,8.825e-03,8.503e-03,
        8.179e-03,7.856e-03,7.532e-03,7.21e-03,6.889e-03,6.571e-03,6.257e-03,5.945e-03,5.639e-03,5.337e-03,
        5.04e-03,4.75e-03,4.466e-03,4.189e-03,3.92e-03,3.658e-03,3.404e-03,3.159e-03,2.922e-03,2.695e-03,
        2.476e-03,2.267e-03,2.067e-03,1.877e-03,1.697e-03,1.526e-03,1.364e-03,1.212e-03,1.07e-03,9.373e-04,
        8.137e-04,6.991e-04,5.935e-04,4.966e-04,4.081e-04,3.277e-04,2.553e-04,1.904e-04,1.328e-04,8.207e-05,
        3.794e-05,0.0,-3.211e-05,-5.877e-05,-8.036e-05,-9.728e-05,-1.099e-04,-1.187e-04,-1.239e-04,-1.26e-04,
        -1.254e-04,-1.223e-04,-1.173e-04,-1.106e-04,-1.026e-04,-9.352e-05,-8.377e-05,-7.363e-05,-6.335e-05,-5.319e-05,
        -4.34e-05,-3.419e-05,-2.577e-05,-1.83e-05,-1.194e-05,-6.827e-06,-3.077e-06,-7.782e-07,0.0
    };
    for (i=0;i<198;i++){
        sumnum += states[i]*znum[i];
        if (i<197) states[i] = states[i+1];
    }
    states[197] = invar;
    sumnum += states[197]*znum[198];
    return sumnum;
}



float DigFil(invar)
float invar;
/******************************************************************************/
/* 2nd Order High Pass Butterworth                                            */
/* Bilinear Transformation with Prewarping                                    */
/* Sample Frequency = 6.000 KHz                                               */
/* Standard Form                                                              */
/* Arithmetic Precision = 9 Digits                                            */
/*                                                                            */
/* Pass Band Frequency = 30.00 Hz                                             */
/*                                                                            */
/******************************************************************************/


{
    float sumnum=0.0, sumden=0.0;  int i=0;
    static float states[2] = {0.0,0.0};
    static float znum[3] = {
        .978030479,
        -1.95606096,
        .978030479
    };
    static float zden[2] = {
        .956543677,
        -1.95557824
    };
    sumnum = sumden = 0.0;
    for (i=0;i<2;i++){
        sumden += states[i]*zden[i];
        sumnum += states[i]*znum[i];
        if (i<1) states[i] = states[i+1];
    }
    states[1] = invar-sumden;
    sumnum += states[1]*znum[2];
    return sumnum;
}



/* EXAMPLE
riff_header='RIFF',
wav_size=size of wave portion
wave_header="WAVE"
fmt_header="fmt"
fmt_chunk_size=16
audio_format=1
num_channels=1
sample_rate=44100
byte_rate=88200
sample_alignment=2
bit_depth=16
data_header="data"
data_bytes=15987456
*/
typedef struct wav_header {
    // RIFF Header
    char riff_header[4]; // Contains "RIFF"
    int wav_size; // Size of the wav portion of the file, which follows the first 8 bytes. File size - 8
    char wave_header[4]; // Contains "WAVE"
    
    // Format Header
    char fmt_header[4]; // Contains "fmt " (includes trailing space)
    int fmt_chunk_size; // Should be 16 for PCM
    short audio_format; // Should be 1 for PCM. 3 for IEEE Float
    short num_channels;
    int sample_rate;
    int byte_rate; // Number of bytes per second. sample_rate * num_channels * Bytes Per Sample
    short sample_alignment; // num_channels * Bytes Per Sample
    short bit_depth; // Number of bits per sample
    /////
   
    // Data
    char data_header[4]; // Contains "data"
    int data_bytes; // Number of bytes in data. Number of samples * num_channels * sample byte size
    // uint8_t bytes[]; // Remainder of wave file is bytes
} wav_header;

/* run this program using the console pauser or add your own getch, system("pause") or input loop */
 
unsigned int streamIIRFilter128_v2(unsigned int bitOfStream, unsigned int* pToOutBuffer);
unsigned int streamIIRFilter64_v2(unsigned int bitOfStream, unsigned int* pToOutBuffer);
void filter64_v3(unsigned long long bitStream,   int* buffer);
void filter128_v3(unsigned long long bitStream,   int* buffer);
void filter64_v4(unsigned long long bitStream,   int* buffer);
void filter64_v5 (unsigned long long bitStream,  int* buffer);
void decimateBlock( int* input,   short* output, unsigned int outputSize);
void cic_filter(unsigned long long bitStream, int* out);
void filter128OfBlock(unsigned char* inputStream,  int* outputData,unsigned int bytesOfStreamPerBlock);
void filter64OfBlock(unsigned char* inputStream, int* outputData,unsigned int bytesOfStreamPerBlock);
void deltaSigmaToInt(unsigned long long bitStream, int* out);
void cic_4x4 (unsigned long long bitStream, int* out);
void cicThridOrder(unsigned long long bitStream, int* out);

int main(int argc, char *argv[]) {
	FILE* rawIntegersFile;
	FILE* myAudioFile;
	FILE* templateFile;
	FILE* bitStreamFile;
	FILE* filteredFile;	
	wav_header audio_header;
	float fl1,fl2,fltemp;
	
	unsigned long long bitStreamBuffer[256];
	 int filteredBuffer[8200];
	 int rawIntegerBuffer[8200];
	short waveBuffer[8292];
	unsigned long long  *auxPtrToBitStream;
    int  *auxPtrToFilteredBuffer;
    int *auxPtrToRawIntegers;
	//amount of 32bit words per block 
    const unsigned int intWordsPerBlock = 8192;
    //amount of 64bit words of bitstream per block 
    const unsigned int bsLongPerBlock = 128;
    //
    unsigned long long bitStreamLength;
    unsigned int wholeBsLongBlocks = 0;
    //bitStreamBuffer = malloc( (bsLongPerBlock*8));
    ////filteredBuffer  = malloc((intWordsPerBlock*4));
     unsigned int fastIntAudioDataLength;
	//open template of WAVE file header into structure
	templateFile = fopen("wav_header","rb");
	fread(&audio_header, sizeof(wav_header), 1, templateFile);
    fclose(templateFile);
    //---------------DBG-{
     
    //-------------------DBG-}
    //open bs file
    bitStreamFile = fopen("stream1.bin","rb");
    if(bitStreamFile == NULL){
    	printf("Can`t open file stream1.bin");
    	return -1;
	}
 
    //read bitstream file size
    fseek(bitStreamFile,0,SEEK_END);
    bitStreamLength = ftell(bitStreamFile);
    fseek(bitStreamFile,0,SEEK_SET);
    //how many whole blocks contains a bs file
    wholeBsLongBlocks = bitStreamLength / (bsLongPerBlock << 3); //mult by 8 
    //open temporaty file
    filteredFile = fopen("filtered","wb");
    rawIntegersFile = fopen("rawnumbers","wb");
    
    
		
	     
	for (int a1=0; a1<wholeBsLongBlocks; a1++) {
		//initializing auxialary pointers
	     auxPtrToBitStream = bitStreamBuffer;
	     auxPtrToFilteredBuffer = filteredBuffer;
	     auxPtrToRawIntegers = rawIntegerBuffer;
		//reading all the bs file by blocks
		//1)read chunk into input buffer
		fread(bitStreamBuffer,bsLongPerBlock,8,bitStreamFile);
 
		
		//2.2)iterate the block and processing data
		for (int y1=0; y1<bsLongPerBlock; y1++) {
		      //a) processing 64bits of BS
			     filter128_v3(*auxPtrToBitStream, auxPtrToFilteredBuffer);
			    //cicThridOrder(*auxPtrToBitStream, auxPtrToFilteredBuffer);
			  	 deltaSigmaToInt(*auxPtrToBitStream, auxPtrToRawIntegers);
			  //b)increment pointers
			  auxPtrToBitStream++;
			  auxPtrToRawIntegers += 64;
			  auxPtrToFilteredBuffer += 64;
		}
		
		//3)Save results (block) into temp file
		fwrite(filteredBuffer,intWordsPerBlock, 4, filteredFile);
		fwrite(rawIntegerBuffer,intWordsPerBlock, 4, rawIntegersFile);
	}
	 	
	 //calculating amount of 32-bit words in high speed data 384kHz
	 fastIntAudioDataLength = intWordsPerBlock * wholeBsLongBlocks;
	
	fclose(filteredFile);
	//open again in read only mode
	filteredFile = fopen("filtered","rb");
	//create audio file
	myAudioFile = fopen("processed.wav","wb");

	audio_header.byte_rate = 22050;//12000;
	audio_header.sample_rate = 11025;// 6000;
	audio_header.data_bytes = (fastIntAudioDataLength >> 5); // divided by 64 and divided by 2 (because 16bit wave)
	//write wave header
	fwrite(&audio_header, sizeof(wav_header),1,myAudioFile);
	
	for ( unsigned int k = 0; k < fastIntAudioDataLength; ) { 
		//1)read block into memory
		fread (filteredBuffer, intWordsPerBlock, 4, filteredFile);
		//1.1)post-filtering
		 auxPtrToFilteredBuffer = filteredBuffer;
	    	for (int a9=0; a9<intWordsPerBlock; a9++) {
			
			 auxPtrToFilteredBuffer[a9] = (unsigned int) DigFil3K_1((float)auxPtrToFilteredBuffer[a9]);
			//auxPtrToFilteredBuffer[a9] =   auxPtrToFilteredBuffer[a9];
		} 
		//2)decimate block
		decimateBlock(filteredBuffer,waveBuffer, (intWordsPerBlock >> 6));
		//3) write block in wave file
		fwrite (waveBuffer,256,1,myAudioFile);
		k += intWordsPerBlock;
		
		
	}
	
	fclose(myAudioFile);
	fclose(filteredFile);
	fclose(rawIntegersFile);
    ///free memory
    //free(bitStreamBuffer);
    //free(filteredBuffer);
	return 0;
}

void filter128OfBlock(unsigned char* inputStream,   int* outputData,unsigned int bytesOfStreamPerBlock) {
	 
	unsigned char inByte=0;
	const char msk = 0x01;
	for (int a3=0; a3 < bytesOfStreamPerBlock; a3++) {
		inByte = *inputStream; 
		 
			streamIIRFilter128_v2((unsigned int)(inByte & msk), outputData);
			inByte >>= 1;
			outputData++;
			streamIIRFilter128_v2((unsigned int)(inByte & msk), outputData);
			inByte >>= 1;
			outputData++;
			streamIIRFilter128_v2((unsigned int)(inByte & msk), outputData);
			inByte >>= 1;
			outputData++;
			streamIIRFilter128_v2((unsigned int)(inByte & msk), outputData);
			inByte >>= 1;
			outputData++;
			streamIIRFilter128_v2((unsigned int)(inByte & msk), outputData);
			inByte >>= 1;
			outputData++;
			streamIIRFilter128_v2((unsigned int)(inByte & msk), outputData);
			inByte >>= 1;
			outputData++;
			streamIIRFilter128_v2((unsigned int)(inByte & msk), outputData);
			inByte >>= 1;
			outputData++;
			streamIIRFilter128_v2((unsigned int)(inByte & msk), outputData);
			inByte >>= 1;
			outputData++;
			
	 
		inputStream++;
	}
}

void filter64OfBlock(unsigned char* inputStream,   int* outputData,unsigned int bytesOfStreamPerBlock) {
	unsigned int* ptr = outputData; 
	unsigned char inByte=0;
	const char msk = 0x01;
	for (int a3=0; a3 < bytesOfStreamPerBlock; a3++) {
		inByte = *inputStream; 
		for (int a4=0; a4<8; a4++ ) {
			streamIIRFilter64_v2((unsigned int)(inByte & msk), ptr);
			inByte >>= 1;
			ptr++;
		}
		inputStream++;
	}
}

void decimateBlock( int* input,   short* output, unsigned int outputSize){
	 
	int avg;
 float sample;
 float res;
   for(;outputSize > 0;){
    //avg=0;
      /*for (int x2=0;x2<64;x2++) {
      	avg += input[x2];
	  }*/
	 //avg <<= 3;
	 avg = input[0];
	 
	 sample = (float)avg;
 
   	
   	///sample = sample*1024;
  	 
	res = DigFil(sample);
	res = res / 2;
	*output = (short)res;
  	input += 64;
  	output++;
  	//if(outputSize & 1 ){
  		
	 // }
  	outputSize--;
  }
}
//{{{{{{{{{{{
//55555555555555555555555555555555555555555555555555555555555555555
/**the filter get a chunk (64bit) of bit stream
 and store result (each bit) into 32bit array**/ 
void filter64_v5 (unsigned long long bitStream,  int* buffer) {
	
	unsigned long long bitToProcess = 0;
	unsigned long long mask = 1;
	unsigned int acc = 0;
	static unsigned long long window = 0; 
	short idx = 0;
	//24 bits
		const unsigned int coefs[]={
	12345, 
	18193, 
	25249, 
	33601, 
	43325, 
	54480, 
	67105, 
	81218, 
	96816, 
	113868, 
	132318, 
	152082, 
	173050, 
	195084, 
	218021, 
	241672, 
	265828, 
	290259, 
	314719, 
	338949, 
	362684, 
	385651, 
	407581, 
	428208, 
	447276, 
	464546, 
	479795, 
	492825, 
	503464, 
	511571, 
	517036, 
	519788, 
	519788, 
	517036, 
	511571, 
	503464, 
	492825, 
	479795, 
	464546, 
	447276, 
	428208, 
	407581, 
	385651, 
	362684, 
	338949, 
	314719, 
	290259, 
	265828, 
	241672, 
	218021, 
	195084, 
	173050, 
	152082, 
	132318, 
	113868, 
	96816, 
	81218, 
	67105, 
	54480, 
	43325, 
	33601, 
	25249, 
	18193, 
	12345,  };
	
	for (int bitCount=0; bitCount < 64; bitCount++) {
		//extracting low bit
	     bitToProcess = bitStream & 1;	
	     //shifting window
		window <<= 1;
		//apply new bit
	    window |= bitToProcess;
	    mask = 1;
	    idx = 0;
		acc = 0;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
			if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;	
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		//store result
	 
		*buffer = (65535 - (acc >> 9));
		//increment pointer (to point to next word)
		buffer++;
	
		//shift BitStr to have next bit al low bit for processing
		bitStream >>= 1;
	}
	   
	 
}

/**the filter get a chunk (64bit) of bit stream
 and store result (each bit) into 32bit array**/ 
void filter64_v4 (unsigned long long bitStream,  int* buffer) {
	
	unsigned long long bitToProcess = 0;
	unsigned long long mask = 1;
	float acc = 0;
	static unsigned long long window = 0; 
	short idx = 0;
	//24 bits
		const float coefs[]={
19591.986328125 , 
39138.214843750 , 
58593.035156250 , 
77911.007812500 , 
97047.023437500 , 
115956.367187500 , 
134594.890625000 , 
152919.062500000 , 
170886.093750000 , 
188454.000000000 , 
205581.781250000 , 
222229.406250000 , 
238358.015625000 , 
253929.921875000 , 
268908.750000000 , 
283259.562500000 , 
296948.781250000 , 
309944.468750000 , 
322216.281250000 , 
333735.531250000 , 
344475.312500000 , 
354410.562500000 , 
363518.062500000 , 
371776.562500000 , 
379166.750000000 , 
385671.406250000 , 
391275.281250000 , 
395965.312500000 , 
399730.562500000 , 
402562.187500000 , 
404453.656250000 , 
405400.468750000 , 
405400.468750000 , 
404453.625000000 , 
402562.156250000 , 
399730.500000000 , 
395965.250000000 , 
391275.187500000 , 
385671.281250000 , 
379166.656250000 , 
371776.437500000 , 
363517.937500000 , 
354410.406250000 , 
344475.125000000 , 
333735.343750000 , 
322216.062500000 , 
309944.250000000 , 
296948.562500000 , 
283259.312500000 , 
268908.500000000 , 
253929.671875000 , 
238357.750000000 , 
222229.140625000 , 
205581.500000000 , 
188453.718750000 , 
170885.781250000 , 
152918.734375000 , 
134594.546875000 , 
115956.007812500 , 
97046.648437500 , 
77910.625000000 , 
58592.644531250 , 
39137.812500000 , 
19591.572265625 ,  };
	
	for (int bitCount=0; bitCount < 64; bitCount++) {
		//extracting low bit
	     bitToProcess = bitStream & 1;	
	     //shifting window
		window <<= 1;
		//apply new bit
	    window |= bitToProcess;
	    mask = 1;
	    idx = 0;
		acc = 0;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
			if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;	
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		//store result
		acc = acc/256.000000000;
		*buffer = (unsigned int)acc;
		//increment pointer (to point to next word)
		buffer++;
	
		//shift BitStr to have next bit al low bit for processing
		bitStream >>= 1;
	}
	   
	 
}
//}}}}}}}
/**the filter get a chunk (64bit) of bit stream
 and store result (each bit) into 32bit array**/ 
void filter64_v3 (unsigned long long bitStream, int* buffer) {
	
	unsigned long long bitToProcess = 0;
	unsigned long long mask = 1;
	unsigned int acc =0;
	static unsigned long long window = 0; 
	short idx = 0;
		const unsigned int coefs[]={24,36,49,66,85,106,131,159,189,222,258,297,338,381,
	426,472,519,567,615,662,709,753,796,836,874,907,937,963,983,999,1010,1015,1015,
	1010,999,983,963,937,907,874,836,796,753,709,662,615,567,519,472,426,381,338,297,
	258,222,189,159,131,106,85,66,49,36,24};
	
	for (int bitCount=0; bitCount < 64; bitCount++) {
		//extracting low bit
	     bitToProcess = bitStream & 1;	
	     //shifting window
		window <<= 1;
		//apply new bit
	    window |= bitToProcess;
	    mask = 1;
	    idx = 0;
		acc = 0;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
				if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
				if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
				if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		if (window & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		//store result
		*buffer = acc;;
		//increment pointer (to point to next word)
		buffer++;
	
		//shift BitStr to have next bit al low bit for processing
		bitStream >>= 1;
	}
	   
	 
}

/**the filter get a chunk (64bit) of bit stream
 and store result (each bit) into 32bit array**/ 
void filter128_v3(unsigned long long bitStream,  int* buffer){
	
	unsigned long long bitToProcess = 0;
	unsigned long long mask = 1;
	unsigned int acc =0;
	unsigned long long carry = 0;
	static unsigned long long windowLow = 0;
	static unsigned long long windowHigh = 0; 
	short idx = 0;
const unsigned int coefs[128]={1,2,4,6,8,11,14,17,21,26,31,37,44,52,61,71,82,94,108,122,
		138,156,174,194,216,238,262,287,313,341,369,399,430,461,493,526,560,594,628,663,697,
		732,767,801,834,868,900,931,962,991,1019,1046,1071,1094,1116,1136,1153,1169,1183,
		1195,1204,1210,1216,1218,1218,1216,1210,1204,1195,1183,1169,1153,1136,1116,1094,
		1071,1046,1019,991,962,931,900,868,834,801,767,732,697,663,628,594,560,526,493,
		461,430,399,369,341,313,287,262,238,216,194,174,156,138,122,108,94,82,71,61,52,
		44,37,31,26,21,17,14,11,8,6,4,2,1 };
	
	for (int bitCount=0; bitCount < 64; bitCount++) {
		//extracting low bit
	     bitToProcess = bitStream & 1;	
		//extracting high bit from low word
		carry = windowLow & 0x8000000000000000;
		carry >>= 63;
		//shift, apply to  high word 
		windowHigh <<= 1;
		windowHigh |= carry;
		//shift low window and apply bit from bit stream 
		windowLow <<= 1;
		windowLow |= bitToProcess;

	    mask = 1;
	    idx = 0;
		acc = 0;
		if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
			if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowLow & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		///////////high word
		mask=1;
			if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
	if (windowHigh & mask) {
			acc +=coefs[idx];
		};
		mask <<= 1;
		idx++;
		//store result
		*buffer = acc;
		//increment pointer (to point to next word)
		buffer++;
///???????????????????????????????????????????????????????????????????
		//shift BitStr to have next bit al low bit for processing
		bitStream >>= 1;
	}
	   
	 
}
 
 

/* 128 Tap Low Pass Kaiser                                                 
  Finite Impulse Response                                                     
  Sample Frequency = 384.0 KHz                                                
  Standard Form                                                               
  Arithmetic Precision = 4 Digits                                                                                                                          
  Pass Band Frequency = 3.000 KHz  */

unsigned int streamIIRFilter64_v2(unsigned int bitOfStream, unsigned int* pToOutBuffer){

	static unsigned long long bits = {0};
	unsigned int result=0;
	unsigned long long bitMask = 1;
 
	const unsigned int coefs[]={24,36,49,66,85,106,131,159,189,222,258,297,338,381,
	426,472,519,567,615,662,709,753,796,836,874,907,937,963,983,999,1010,1015,1015,
	1010,999,983,963,937,907,874,836,796,753,709,662,615,567,519,472,426,381,338,297,
	258,222,189,159,131,106,85,66,49,36,24};
	
		bitOfStream &= 0x00000001;
	    bits 	<<= 1;
	    bits |= bitOfStream;
	    for (int a=0; a<64; a++) {
		 
	    	if (bitMask & bits) {
	            result += coefs[a];		
			}
			bitMask <<= 1;
		}
		
		*pToOutBuffer = result;
		
		//printf("%X, %X \n",bits[0], bits[1]);
	//	printf("%d \n", result);
		
}

//converts bit-stream into 32-bit integer chunk of data
//This function save previous results in an static array
/* 128 Tap Low Pass Hamming                                                   
  Finite Impulse Response                                                     
  Sample Frequency = 384.0 KHz                                                
  Standard Form                                                               
  Arithmetic Precision = 4 Digits                                                                                                                          
  Pass Band Frequency = 3.000 KHz  */
  //sum all the coefs equals 65534
unsigned int streamIIRFilter128_v2(unsigned int bitOfStream, unsigned int* pToOutBuffer) {
	
	static unsigned long long bits0 = 0;
	static unsigned long long bits1 = 0;
	unsigned int counterIndex = 0;
	unsigned long long state = 0;
	
	unsigned int result=0;
	const unsigned int coefs[128]={1,2,4,6,8,11,14,17,21,26,31,37,44,52,61,71,82,94,108,122,
		138,156,174,194,216,238,262,287,313,341,369,399,430,461,493,526,560,594,628,663,697,
		732,767,801,834,868,900,931,962,991,1019,1046,1071,1094,1116,1136,1153,1169,1183,
		1195,1204,1210,1216,1218,1218,1216,1210,1204,1195,1183,1169,1153,1136,1116,1094,
		1071,1046,1019,991,962,931,900,868,834,801,767,732,697,663,628,594,560,526,493,
		461,430,399,369,341,313,287,262,238,216,194,174,156,138,122,108,94,82,71,61,52,
		44,37,31,26,21,17,14,11,8,6,4,2,1 };
		bitOfStream &= 0x00000001;
		
		//1) shifting high word
		bits1 <<= 1;
		//2)Is the MSB in low word = 1  ?
		 state  = bits0 & 0x8000000000000000; //extract MSB
		 state >>= 63; //move to LSB
		 bits1 |= state; //apply to high word
		 //3) shft low word and apply input bit-stream
		 bits0 <<= 1;
		 bits0 |= bitOfStream;
		//processing first 64bits
	 
		state = 1;
		for (int a=0; a<64; a++) {
	    	if (state & bits0) {
	            result += coefs[counterIndex];		
			}
			state <<= 1;
			counterIndex++;
		}
		//processing last 64bit
		state = 1;
		for (int b=0; b<64; b++) {
		 
	    	if (state & bits1) {
	            result += coefs[counterIndex];		
			}
			state <<= 1;
			counterIndex++;
		}
		
		*pToOutBuffer = result;
		
	 //	printf("%d \n", result );
//	printf("%llx, %llx \n",bits0,bits1);
		
}

void cic_filter(unsigned long long bitStream, int* out) {
	//static int* combDelayBasePointer=0; //full pointers to memory
	unsigned char comb8IdxIn, comb8IdxOut =0;//contains low 8-bit of addresses
	static int combSamples [64]={0}; //delay buffer
	static int acc=0;
	const unsigned long long mask = 1;
	int  comb=0;
	//init low parts
	comb8IdxIn = 0;
	comb8IdxOut = 4; //size of int = 4bytes
	
	for (int x1=0; x1 < 64; x1++) {
			// processing of a bit from bit stream
	 
		//2)Add the lowest bit to acc:
	      acc += bitStream & mask;
	    //3)Send integrator to a comb:
		comb = acc - combSamples [ (comb8IdxOut >> 2)];
		//4)Assign the current value
		combSamples[(comb8IdxIn >> 2)] = acc;
		 //5) Shift low parts of pointers
		 comb8IdxIn += 4;
		 comb8IdxOut += 4;
		 //6)save new sample
		 *out = comb << 10;
		 //7)increment out opointer
		 out++;
		//8) shift bits of bitstream
		bitStream >>= 1;
	}

	
}


///// converting bit-stream into integer #########
void deltaSigmaToInt(unsigned long long bitStream, int* out) {
	//static int* combDelayBasePointer=0; //full pointers to memory
	const unsigned long long mask = 1;
	for (int x1=0; x1 < 64; x1++) {
		// processing of a bit from bit stream
	 
		//1)Add the lowest bit to acc:
	      if ( bitStream & mask) {
	      	//when 1 in bit-stream, asign maximum positive
	      	 *out =  1000000000;
		  } else {
		  	//when 0 in bit-stream , assign minimum value
		  	 *out = -1000000000;
		  }
		 //2)increment out opointer
		 out++;
		//4) shift bits of bitstream
		bitStream >>= 1;
	}
	
}

void cicThridOrder(unsigned long long bitStream, int* out) {
	//static int* combDelayBasePointer=0; //full pointers to memory
	unsigned static char comb1IdxIn = 0;
	unsigned static char comb1IdxOut = 1;
	unsigned static char comb2IdxIn = 0;
	unsigned static char comb2IdxOut = 1;
	unsigned static char comb3IdxIn = 0;
	unsigned static char  comb3IdxOut =1;
	unsigned static char comb4IdxIn = 0;
	unsigned static char  comb4IdxOut =1;
	unsigned static char comb5IdxIn = 0;
	unsigned static char  comb5IdxOut =1;
	
	static int comb1Samples [64] = {0}; //delay buffer
	static int comb2Samples [64] = {0}; //delay buffer
	static int comb3Samples [64] = {0}; //delay buffer
	static int comb4Samples [64] = {0}; //delay buffer
	static int comb5Samples [64] = {0}; //delay buffer
	
	int  comb1 = 0;
	int  comb2 = 0;
	int  comb3 = 0;
		int  comb4 = 0;
		int  comb5 = 0;
	
	static int acc1 = 0;
	static int acc2 = 0;
	static int acc3 = 0;
	static int acc4 = 0;
	static int acc5 = 0;

	const unsigned long long mask = 1;
	#define DELAY_SIZE 64  // Comb delay size -1 
 
	
	for (int x1=0; x1 < 64; x1++) {
			// processing of a bit from bit stream
	  /****first filter stage***/
	  /***acc1**/
		//1)Accumulators section:
	      acc1 += bitStream & mask;
	       
	      acc2 += acc1;
	       
	      acc3 += acc2;
	      
	      acc4 += acc3;
	      
	      acc5 += acc4;
	      //2) combs section:
	      //A) first stage:
	      
	      comb1 = acc3 - comb1Samples [ comb1IdxOut]; //comb out calculating
	      comb1Samples[comb1IdxIn] = acc3; //push input value to delay line
	      comb1IdxIn++;  //update indexes
		  comb1IdxOut++;  //update indexes
		  //wrap around implementation
		  comb1IdxIn = comb1IdxIn % DELAY_SIZE;
		  comb1IdxOut = comb1IdxOut % DELAY_SIZE;
		  //comb1 <<= 8;
		 //B) Second stage:
	      comb2 = comb1 - comb2Samples [ comb2IdxOut]; //comb out calculating
	      comb2Samples[comb2IdxIn] = comb1; //push input value to delay line
	      comb2IdxIn++;  //update indexes
		  comb2IdxOut++;  //update indexes
		  //wrap around implementation
		  comb2IdxIn = comb2IdxIn % DELAY_SIZE;
		  comb2IdxOut = comb2IdxOut % DELAY_SIZE;
		  //C) Thrid:
	      comb3 = comb2 - comb3Samples [ comb3IdxOut]; //comb out calculating
	      comb3Samples[comb3IdxIn] = comb2; //push input value to delay line
	      comb3IdxIn++;  //update indexes
		  comb3IdxOut++;  //update indexes
		  //wrap around implementation
		  comb3IdxIn = comb3IdxIn % DELAY_SIZE ;
		  comb3IdxOut = comb3IdxOut % DELAY_SIZE;
		  //D) forth
		  comb4 = comb3 - comb3Samples [ comb4IdxOut]; //comb out calculating
	      comb4Samples[comb4IdxIn] = comb3; //push input value to delay line
	      comb4IdxIn++;  //update indexes
		  comb4IdxOut++;  //update indexes
		  //wrap around implementation
		  comb4IdxIn = comb4IdxIn % DELAY_SIZE ;
		  comb4IdxOut = comb4IdxOut % DELAY_SIZE;
		  //E) fifth
		  comb5 = comb4 - comb5Samples [ comb5IdxOut]; //comb out calculating
	      comb5Samples[comb5IdxIn] = comb4; //push input value to delay line
	      comb5IdxIn++;  //update indexes
		  comb5IdxOut++;  //update indexes
		  //wrap around implementation
		  comb5IdxIn = comb5IdxIn % DELAY_SIZE ;
		  comb5IdxOut = comb5IdxOut % DELAY_SIZE;
		  
		 //$ save new sample
		 *out = comb3>>2;
		 //7)increment out opointer
		 out++;
		//8) shift bits of bitstream
		bitStream >>= 1;
	}
	
}
	
