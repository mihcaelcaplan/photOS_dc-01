/*
 * avi.h
 *
 *  Created on: Dec 2, 2025
 *      Author: mcaplan
 */

#ifndef AVI_H_
#define AVI_H_

#include "ff.h"

/* this file is based on the need to write fast AVIS 
* that means: 
* write the actual video data first 
* then seek back and write headers
* so first we should just skip to 
*/

typedef struct {
    DWORD patch_riff_avi_size;
    DWORD patch_avih_totalframes;
    DWORD patch_movi_size;
    DWORD patch_strh_length;
} patch_info_t;

// typedef struct patches;

//init what we will need here
typedef struct {
    FIL* file_pointer;
    FIL* index_pointer;
    DWORD frame_counter;
    // uint32_t* riff_start;
    uint32_t movie_start;
    uint32_t movie_end;
    uint32_t file_end; // don't know if i need this
} avi_handle_t;

extern avi_handle_t avi_handle; // declare the handle that will be reused

// init the handle, write static headers
void AVI_Init( FIL* fp, uint32_t img_width, uint32_t img_height, uint32_t img_size );

// add the jpeg frame to the the movie
void AVI_AddFrame( uint32_t* img_ptr, uint32_t img_size);

// this is the patch method that will add headers and index
void AVI_Patch( void );


// definitions for RIFF header for use in AVI container
typedef uint32_t DWORD;
typedef uint32_t LONG;
typedef uint16_t WORD;
typedef uint8_t BYTE;

typedef struct {
    DWORD dwRIFF;
    DWORD dwSize;
    DWORD dwFourCC;
} RIFF;

typedef struct {
    DWORD dwFourCC;
    DWORD dwSize;
} CHUNK;

typedef struct {
    DWORD dwList;
    DWORD dwSize;
    DWORD dwFourCC;
} LIST;

typedef struct {
    DWORD dwFourCC;
    DWORD dwSize;

    DWORD dwMicroSecPerFrame;
    DWORD dwMaxBytesPerSec;
    DWORD dwPaddingGranularity;

    DWORD dwFlags;
    DWORD dwTotalFrames;
    DWORD dwInitialFrames;
    DWORD dwStreams;
    DWORD dwSuggestedBufferSize;

    DWORD dwWidth;
    DWORD dwHeight;

    DWORD dwReserved[4];
} MainAVIHeader;

typedef struct {
    DWORD dwFourCC;
    DWORD dwSize;

    DWORD fccType;
    DWORD fccHandler;
    DWORD dwFlags;
    WORD  wPriority;
    WORD  wLanguage;
    DWORD dwInitialFrames;
    DWORD dwScale;
    DWORD dwRate;
    DWORD dwStart;
    DWORD dwLength;
    DWORD dwSuggestedBufferSize;
    DWORD dwQuality;
    DWORD dwSampleSize;
    // do i need rcFrame here? yes i think
    DWORD dwRcStart;
    DWORD dwRcEnd;

} AVIStreamHeader;

typedef struct {
    DWORD dwFourCC;
    DWORD dwSize;

    DWORD biSize;
    DWORD biWidth;
    DWORD biHeight;
    WORD  biPlanes;
    WORD  biBitCount;
    DWORD biCompression;
    DWORD biSizeImage;
    DWORD biXPelsPerMeter;
    DWORD biYPelsPerMeter;
    DWORD biClrUsed;
    DWORD biClrImportant;
} BITMAPINFOHEADER;

/* doesn't really help that much so not using*/
typedef struct {
    DWORD ckid;
    DWORD dwFlags;
    DWORD dwChunkOffset;
    DWORD dwChunkLength;
} AVIINDEXENTRY;


// index arrays


/* schematic but unused*/
// typedef struct {
//     RIFF riff_AVI;
//     LILST hdrl;
//     MainAVIHeader avih;
//     LIST strl;
//     AVIStreamHeader strh;
//     BITMAPINFOHEADER strf;
//     // LIST odml;
//     // ODMLExtendedAVIheader dmlh;
//     LIST movi;
//     CHUNK movi_data;
// } avi_file;


#endif /* AVI_H_ */
