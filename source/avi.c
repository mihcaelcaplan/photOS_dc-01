/*
 * avi.c
 *
 *  Created on: Dec 2, 2025
 *      Author: mcaplan
 */

#include "avi.h"
#include "ff.h"
#include "fsl_common.h"
#include "fsl_debug_console.h"

//  make a global handle TODO: remember to clean this after f_closing a completed avi :)

#define MAKEFOURCC(ch0, ch1, ch2, ch3) \
    ((DWORD)(uint8_t)(ch0) | ((DWORD)(uint8_t)(ch1) << 8) | \
    ((DWORD)(uint8_t)(ch2) << 16) | ((DWORD)(uint8_t)(ch3) << 24))

avi_handle_t avi_handle = {
    .file_pointer = 0,
    .frame_counter = 0
};


void write_DWORD(DWORD data){
    uint32_t bytes_to_write = 4; // dword len
    uint32_t bytes_written = 0;
    
    FRESULT err = f_write(avi_handle.file_pointer, &data, bytes_to_write, &bytes_written);
    if(err != 0){
		PRINTF("f_write err: %d\r\n", bytes_written, bytes_to_write);
    }

    assert(bytes_to_write == bytes_written);
}

void write_WORD(WORD data){
    
	uint32_t bytes_to_write = 2; // word len
	uint32_t bytes_written = 0;
    
    f_write(avi_handle.file_pointer, &data, bytes_to_write, &bytes_written);

    assert(bytes_to_write == bytes_written);

}

// static initialize the fixed parts of the headers
RIFF riff_avi = {
    .dwRIFF = MAKEFOURCC('R', 'I', 'F', 'F'),
    .dwSize = 0xDEADBEEF, // this must be filled
    .dwFourCC = MAKEFOURCC('A','V','I',' ')
};

LIST hdrl = {
    .dwList = MAKEFOURCC('L','I','S','T'),
    .dwSize = 208,
    .dwFourCC = MAKEFOURCC('h','d','r','l')
};

MainAVIHeader avih = {
    .dwFourCC = MAKEFOURCC('a','v','i','h'),
    .dwSize = 56,
    .dwMicroSecPerFrame = 70000,
    .dwMaxBytesPerSec = 44236800, // 1920px * 1920px *20 frames/sec * 3 Bpp *1/5 compression (assumes )
    .dwPaddingGranularity = 0,
    .dwFlags = 0,
    .dwTotalFrames = 0xDEADBEEF,
    .dwInitialFrames = 0,
    .dwStreams = 1,
    .dwSuggestedBufferSize = 20000,
    .dwWidth = 1920,
    .dwHeight = 1920,
    .dwReserved = {0}
};

LIST strl = {
    .dwList = MAKEFOURCC('L','I','S','T'),
    // .dwSize = , // stream header (strh) + stream format (strf) doesnt seem to have one in my avi??
    .dwFourCC = MAKEFOURCC('s','t','r','l')
};

CHUNK strh = {
    .dwFourCC = MAKEFOURCC('s','t','r','h'),
    .dwSize = 56
};

BITMAPINFOHEADER strf = {
    .dwFourCC = MAKEFOURCC('v','i','d','s'),
    .dwSize = 40, // bitmapinfoheader size
    .biSize = 40,
    .biWidth = 0, //set in init
    .biHeight = 0, //set in init
    .biPlanes = 1,
    .biBitCount = 24,
    .biCompression = MAKEFOURCC('M','J','P','G'),
    .biSizeImage = 0, //set in init
    .biXPelsPerMeter = 0,
    .biYPelsPerMeter = 0,
    .biClrUsed = 0,
    .biClrImportant = 0
};

LIST movi = {
    .dwList = MAKEFOURCC('L','I','S','T'),
    .dwSize = 0xDEADBEEF,
    .dwFourCC = MAKEFOURCC('m','o','v','i')
};

CHUNK movie_chunk = {
    .dwFourCC = MAKEFOURCC('0','1','w','b'), // c? ??
    .dwSize = 0 //set at encode time
};

patch_info_t patches;


/* This should be very light as it needs to be done before the init */
void AVI_Init(FIL* fp, uint32_t img_width, uint32_t img_height, uint32_t img_size){

    //set the handle to the address of the opened file
    avi_handle.file_pointer = fp;

    // write all the fixed headers so that we can get to the point in the file to start adding frames :)
    // record the 2nd pass patch locations in the

    // write non-fixed args to the right structures
    strf.biWidth = img_width;
    strf.biHeight = img_height;
    strf.biSizeImage = img_size;

    // do the file init :)
    write_DWORD(riff_avi.dwRIFF);
    patches.patch_riff_avi_size = f_tell(fp);
    write_DWORD(riff_avi.dwSize);
    write_DWORD(riff_avi.dwFourCC);

    write_DWORD(avih.dwFourCC);
    write_DWORD(avih.dwSize);
    write_DWORD(avih.dwMicroSecPerFrame);
    write_DWORD(avih.dwMaxBytesPerSec);
    write_DWORD(avih.dwPaddingGranularity);
    write_DWORD(avih.dwFlags);
    patches.patch_avih_totalframes = f_tell(fp);
    write_DWORD(avih.dwTotalFrames); 
    write_DWORD(avih.dwInitialFrames);
    write_DWORD(avih.dwStreams);
    write_DWORD(avih.dwSuggestedBufferSize);
    write_DWORD(avih.dwWidth);
    write_DWORD(avih.dwHeight);
    write_DWORD(avih.dwHeight);
    write_DWORD(avih.dwReserved[0]); // just 0s 
    write_DWORD(avih.dwReserved[1]); // just 0s 
    write_DWORD(avih.dwReserved[2]); // just 0s 
    write_DWORD(avih.dwReserved[3]); // just 0s 
    
    write_DWORD(strl.dwList); 
    write_DWORD(strl.dwFourCC); 
    
    write_DWORD(strh.dwFourCC); 
    write_DWORD(strh.dwSize); 
    
    write_DWORD(strf.dwFourCC); 
    write_DWORD(strf.dwSize); 
    write_DWORD(strf.biSize); 
    write_DWORD(strf.biWidth); 
    write_DWORD(strf.biHeight); 
    write_WORD(strf.biPlanes); 
    write_WORD(strf.biBitCount); 
    write_DWORD(strf.biCompression); 
    write_DWORD(strf.biSizeImage); 
    write_DWORD(strf.biXPelsPerMeter); 
    write_DWORD(strf.biYPelsPerMeter); 
    write_DWORD(strf.biClrUsed); 
    write_DWORD(strf.biClrImportant); 
    
    write_DWORD(movi.dwList); 
    patches.patch_movi_size = f_tell(fp);
    write_DWORD(movi.dwSize); 
    write_DWORD(movi.dwFourCC); 
    avi_handle.movie_start = f_tell(fp);

}


// add the jpeg frame to the the movie
void AVI_AddFrame( uint32_t* img_ptr, uint32_t img_size ){
    
    uint32_t cur_frame_offset = f_tell(avi_handle.file_pointer) - avi_handle.movie_start;
    
    write_DWORD(movie_chunk.dwFourCC);
    write_DWORD(img_size);
    
    // create index chunk 
    AVIINDEXENTRY index = {
        .ckid = movie_chunk.dwFourCC,
        .dwFlags = 0x10,
        .dwChunkOffset = cur_frame_offset,
        .dwChunkLength = img_size,
    };
    
    // write data 
    uint32_t bytesWritten = 0;
    uint32_t* write_pos = img_ptr;
    uint32_t bytesRemain = img_size;
    while (bytesRemain > 0){
        f_write(avi_handle.file_pointer, write_pos, bytesRemain, &bytesWritten);
        bytesRemain -= bytesWritten;
        write_pos += bytesWritten;
    }

    // write index chunk to tmp file
    f_write(avi_handle.index_pointer, &index.ckid, 4, 0);
    f_write(avi_handle.index_pointer, &index.dwFlags, 4, 0);
    f_write(avi_handle.index_pointer, &index.dwChunkOffset, 4, 0);
    f_write(avi_handle.index_pointer, &index.dwChunkLength, 4, 0);

    
    avi_handle.frame_counter++;

}

// this is the patch method that will add headers and index
void AVI_Patch( void ){
    // store end of movie section
    avi_handle.movie_end = f_tell(avi_handle.file_pointer);
    uint32_t total_frames = avi_handle.frame_counter;
    
    // write index
    write_DWORD(MAKEFOURCC('i','d','x','1'));
    
    uint32_t index_len = 16 * total_frames;
    write_DWORD(index_len);

    // flush index
    f_close(avi_handle.index_pointer);
    f_open(avi_handle.index_pointer, "index.tmp", FA_READ);

    // copy over the index
    uint32_t total_copied = 0;
    #define COPY_BUFF_SIZE 4096
    uint8_t copy_buffer[COPY_BUFF_SIZE];
    uint32_t bytes_to_read = 0;
    uint32_t bytes_read = 0;
    uint32_t bytes_written = 0;

    while(total_copied < index_len){
        bytes_to_read = index_len - total_copied;
        if (bytes_to_read > COPY_BUFF_SIZE) {
            bytes_to_read = COPY_BUFF_SIZE;
        }
        
        FRESULT indx_read = f_read(avi_handle.index_pointer, copy_buffer, bytes_to_read, &bytes_read);
        FRESULT indx_copy = f_write(avi_handle.file_pointer, copy_buffer, bytes_read, &bytes_written);

        total_copied += bytes_written;

    }
    
    // delete the tmp file
    f_unlink("index.tmp");

    // store end of file inc. index
    avi_handle.file_end = f_tell(avi_handle.file_pointer);

    // calculate total frames and patch
    
    f_lseek(avi_handle.file_pointer, patches.patch_avih_totalframes);
    write_DWORD(total_frames);
    
    // calculate movie size and patch
    uint32_t movie_size = avi_handle.movie_end - patches.patch_movi_size - 4;
    
    f_lseek(avi_handle.file_pointer, patches.patch_movi_size);
    write_DWORD(movie_size);
    
    // calculate total riff size and patch
    uint32_t riff_size = avi_handle.file_end - 8;
    
    f_lseek(avi_handle.file_pointer, patches.patch_riff_avi_size);
    write_DWORD(riff_size);
}
