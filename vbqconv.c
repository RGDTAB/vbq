/*

Copyright (c) 2023, Dominic Szablewski - https://phoboslab.org
Copyright (c) 2025, Raymond DiDonato
SPDX-License-Identifier: MIT


Command line tool to convert FLAC, MP3, WAV to QOA and QOA to WAV

Define QOACONV_HAS_DRMP3 and QOACONV_HAS_DRFLAC for MP3 and FLAC support
 -"dr_mp3.h" (https://github.com/mackron/dr_libs/blob/master/dr_mp3.h)
 -"dr_flac.h" (https://github.com/mackron/dr_libs/blob/master/dr_flac.h)

Compile with: 
	gcc vbqconv.c -std=gnu99 -lm -O3 -o vbqconv

*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

#ifdef QOACONV_HAS_DRMP3
	/* https://github.com/mackron/dr_libs/blob/master/dr_mp3.h */
	#define DR_MP3_IMPLEMENTATION
	#include "dr_mp3.h"
#endif

#ifdef QOACONV_HAS_DRFLAC
	/* https://github.com/mackron/dr_libs/blob/master/dr_flac.h */
	#define DR_FLAC_IMPLEMENTATION
	#include "dr_flac.h"
#endif

#define VBQ_IMPLEMENTATION
#define QOA_RECORD_TOTAL_ERROR
#include "vbq.h"

#define QOACONV_STRINGIFY(x) #x
#define QOACONV_TOSTRING(x) QOACONV_STRINGIFY(x)
#define QOACONV_ABORT(...) \
	printf("Abort at line " QOACONV_TOSTRING(__LINE__) ": " __VA_ARGS__); \
	printf("\n"); \
	exit(1)
#define QOACONV_ASSERT(TEST, ...) \
	if (!(TEST)) { \
		QOACONV_ABORT(__VA_ARGS__); \
	}

#define QOACONV_STR_ENDS_WITH(S, E) (strcmp(S + strlen(S) - (sizeof(E)-1), E) == 0)

#define QOACONV_STR_SAME(S, E) (strcmp(S, E) == 0)

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

/* -----------------------------------------------------------------------------
	WAV reader / writer */

#define QOACONV_CHUNK_ID(S) \
	(((unsigned int)(S[3])) << 24 | ((unsigned int)(S[2])) << 16 | \
	 ((unsigned int)(S[1])) <<  8 | ((unsigned int)(S[0])))

void qoaconv_fwrite_u32_le(unsigned int v, FILE *fh) {
	unsigned char buf[sizeof(unsigned int)];
	buf[0] = 0xff & (v      );
	buf[1] = 0xff & (v >>  8);
	buf[2] = 0xff & (v >> 16);
	buf[3] = 0xff & (v >> 24);
	int wrote = fwrite(buf, sizeof(unsigned int), 1, fh);
	QOACONV_ASSERT(wrote, "Write error");
}

void qoaconv_fwrite_u16_le(unsigned short v, FILE *fh) {
	unsigned char buf[sizeof(unsigned short)];
	buf[0] = 0xff & (v      );
	buf[1] = 0xff & (v >>  8);
	int wrote = fwrite(buf, sizeof(unsigned short), 1, fh);
	QOACONV_ASSERT(wrote, "Write error");
}

unsigned int qoaconv_fread_u32_le(FILE *fh) {
	unsigned char buf[sizeof(unsigned int)];
	int read = fread(buf, sizeof(unsigned int), 1, fh);
	QOACONV_ASSERT(read, "Read error or unexpected end of file");
	return (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
}

unsigned short qoaconv_fread_u16_le(FILE *fh) {
	unsigned char buf[sizeof(unsigned short)];
	int read = fread(buf, sizeof(unsigned short), 1, fh);
	QOACONV_ASSERT(read, "Read error or unexpected end of file");
	return (buf[1] << 8) | buf[0];
}

int qoaconv_wav_write(const char *path, short *sample_data, qoa_desc *desc) {
	unsigned int data_size = desc->samples * desc->channels * sizeof(short);
	unsigned int samplerate = desc->samplerate;
	short bits_per_sample = 16;
	short channels = desc->channels;

	/* Lifted from https://www.jonolick.com/code.html - public domain
	Made endian agnostic using qoaconv_fwrite() */
	FILE *fh = fopen(path, "wb");
	QOACONV_ASSERT(fh, "Can't open %s for writing", path);
	fwrite("RIFF", 1, 4, fh);
	qoaconv_fwrite_u32_le(data_size + 44 - 8, fh);
	fwrite("WAVEfmt \x10\x00\x00\x00\x01\x00", 1, 14, fh);
	qoaconv_fwrite_u16_le(channels, fh);
	qoaconv_fwrite_u32_le(samplerate, fh);
	qoaconv_fwrite_u32_le(channels * samplerate * bits_per_sample/8, fh);
	qoaconv_fwrite_u16_le(channels * bits_per_sample/8, fh);
	qoaconv_fwrite_u16_le(bits_per_sample, fh);
	fwrite("data", 1, 4, fh);
	qoaconv_fwrite_u32_le(data_size, fh);
	fwrite((void*)sample_data, data_size, 1, fh);
	fclose(fh);
	return data_size  + 44 - 8;
}

short *qoaconv_wav_read(const char *path, qoa_desc *desc) {
	FILE *fh = fopen(path, "rb");
	QOACONV_ASSERT(fh, "Can't open %s for reading", path);

	unsigned int container_type = qoaconv_fread_u32_le(fh);
	QOACONV_ASSERT(container_type == QOACONV_CHUNK_ID("RIFF"), "Not a RIFF container");

	unsigned int wav_size = qoaconv_fread_u32_le(fh);
	unsigned int wavid = qoaconv_fread_u32_le(fh);
	QOACONV_ASSERT(wavid == QOACONV_CHUNK_ID("WAVE"), "No WAVE id found");

	unsigned int data_size = 0;
	unsigned int format_length = 0;
	unsigned int format_type = 0;
	unsigned int channels = 0;
	unsigned int samplerate = 0;
	unsigned int byte_rate = 0;
	unsigned int block_align = 0;
	unsigned int bits_per_sample = 0;

	/* Find the fmt and data chunk, skip all others */
	while (1) {
		unsigned int chunk_type = qoaconv_fread_u32_le(fh);
		unsigned int chunk_size = qoaconv_fread_u32_le(fh);

		if (chunk_type == QOACONV_CHUNK_ID("fmt ")) {
			QOACONV_ASSERT(chunk_size == 16 || chunk_size == 18, "WAV fmt chunk size missmatch");

			format_type = qoaconv_fread_u16_le(fh);
			channels = qoaconv_fread_u16_le(fh);
			samplerate = qoaconv_fread_u32_le(fh);
			byte_rate = qoaconv_fread_u32_le(fh);
			block_align = qoaconv_fread_u16_le(fh);
			bits_per_sample = qoaconv_fread_u16_le(fh);

			if (chunk_size == 18) {
				unsigned short extra_params = qoaconv_fread_u16_le(fh);
				QOACONV_ASSERT(extra_params == 0, "WAV fmt extra params not supported");
			}
		}
		else if (chunk_type == QOACONV_CHUNK_ID("data")) {
			data_size = chunk_size;
			break;
		}
		else {
			int seek_result = fseek(fh, chunk_size, SEEK_CUR);
			QOACONV_ASSERT(seek_result == 0, "Malformed RIFF header");
		}
	}

	QOACONV_ASSERT(format_type == 1, "Type in fmt chunk is not PCM");
	QOACONV_ASSERT(bits_per_sample == 16, "Bits per samples != 16");
	QOACONV_ASSERT(data_size, "No data chunk");

	unsigned char *wav_bytes = malloc(data_size);
	QOACONV_ASSERT(wav_bytes, "Malloc for %d bytes failed", data_size);
	int read = fread(wav_bytes, data_size, 1, fh);
	QOACONV_ASSERT(read, "Read error or unexpected end of file for %d bytes", data_size);
	fclose(fh);

	desc->samplerate = samplerate;
	desc->samples = data_size / (channels * (bits_per_sample/8));
	desc->channels = channels;

	return (short*)wav_bytes;
}



/* -----------------------------------------------------------------------------
	MP3 decode wrapper */

#ifdef QOACONV_HAS_DRMP3
	short *qoaconv_mp3_read(const char *path, qoa_desc *desc) {
		drmp3_uint64 samples;

		drmp3_config mp3;
		short* sample_data = drmp3_open_file_and_read_pcm_frames_s16(path, &mp3, &samples, NULL);
		QOACONV_ASSERT(sample_data, "Can't decode MP3");

		desc->samplerate = mp3.sampleRate;
		desc->channels = mp3.channels;
		desc->samples = samples;

		return sample_data;
	}
#endif



/* -----------------------------------------------------------------------------
	FLAC decode wrapper */

#ifdef QOACONV_HAS_DRFLAC
	short *qoaconv_flac_read(const char *path, qoa_desc *desc) {
		unsigned int channels;
		unsigned int samplerate;
		drflac_uint64 samples;
		short* sample_data = drflac_open_file_and_read_pcm_frames_s16(path, &channels, &samplerate, &samples, NULL);
		QOACONV_ASSERT(sample_data, "Can't decode FLAC");

		desc->samplerate = samplerate;
		desc->channels = channels;
		desc->samples = samples;

		return sample_data;
	}
#endif

double *sinc_table = NULL;
unsigned int sinc_zeros = 13;
unsigned int sinc_spz = 256;

void vbqconv_build_sinc_table() {
    long size = sinc_zeros * sinc_spz;
    double fsize = sinc_zeros * sinc_spz;
    sinc_table = QOA_MALLOC(2 * sinc_zeros * sinc_spz * sizeof(double));
    sinc_table[0] = 1.0;
    for (long i = 1; i < size; i++) {
        // Blackman-Harris Window
        double window = 0.35875 - 0.48829 * cos((M_PI * (i + size)) / fsize);
        window += 0.14128 * cos((2.0 * M_PI * (i + size)) / fsize);
        window -= 0.01168 * cos((3.0 * M_PI * (i + size)) / fsize);

        double angle = M_PI * i / ((double) sinc_spz);
        double snc;
        if (angle) {
            snc = (sin(angle)) / angle;
        } else {
            snc = 1.0f;
        }
        sinc_table[i] = window * snc;
        sinc_table[i + sinc_zeros * sinc_spz - 1] = sinc_table[i] - sinc_table[i - 1];
    }
}

// https://ccrma.stanford.edu/~jos/resample/resample.html
void vbqconv_sinc_resample(const short *sample_data, qoa_desc *qoa, unsigned int vbr_len, unsigned int ratio, unsigned int sample_index, short *resample_buffer) {
    double *table_diff = sinc_table + sinc_zeros * sinc_spz;
    double step = VBQ_FFT_LEN / (double) ratio;
    double normalised_ratio = 1.0 / step;
    double filter_step = normalised_ratio;
    filter_step *= sinc_spz;

    for (int c = 0; c < qoa->channels; c++) {
        double offset = step - 1.0;
        for (int s = c; s < vbr_len * ratio  * qoa->channels + c; s += qoa->channels) {
            double out = 0.0;
            long pos = offset;
            double mix = offset - pos;

            double filter_offset = filter_step * mix;
            double range = (sinc_zeros * sinc_spz - filter_offset) / filter_step;
            range = range > pos + sample_index ? pos + sample_index: range;
            for (int n = -range; n < 1; n++) {
                double filter_loc = filter_offset + filter_step * (-n);
                long filter_pos = filter_loc;
                double alpha = filter_loc - filter_pos;

                double weight = sinc_table[filter_pos] + alpha * table_diff[filter_pos];

                int sample = sample_data[(pos + n) * qoa->channels + c];
                if (sample < 0) {
                    out += (sample * weight) / 32768.0;
                } else {
                    out += (sample * weight) / 32767.0;
                }
            }

            mix = 1.0 - mix;
            filter_offset = (filter_step * mix);
            range = (sinc_zeros * sinc_spz - filter_offset) / filter_step;
            range = range > qoa->samples - sample_index - pos - 1 ? qoa->samples - sample_index - pos - 1: range;
            for (int n = 0; n < range; n++) {
                double filter_loc = filter_offset + filter_step * n;
                long filter_pos = filter_loc;
                double alpha = filter_loc - filter_pos;

                double weight = sinc_table[filter_pos] + alpha * table_diff[filter_pos];

                int sample = sample_data[(pos + 1 + n) * qoa->channels + c];
                if (sample < 0) {
                    out += (sample * weight) / 32768.0;
                } else {
                    out += (sample * weight) / 32767.0;
                }
            }

            out = out * normalised_ratio;
            out = out > 1.0 ? 1.0 : out;
            out = out < -1.0 ? -1.0 : out;
            if (out < 0.0) {
                resample_buffer[s] = qoa_clamp_s16(out * 32768.0);
            } else {
                resample_buffer[s] = qoa_clamp_s16(out * 32767.0);
            }
            offset += step;
        }
    }
}


/* -----------------------------------------------------------------------------
	Main */

int main(int argc, char **argv) {
	QOACONV_ASSERT(argc >= 3, "\nUsage: qoaconv in.{wav,mp3,flac,qoa} out.{wav,qoa}")

	qoa_desc desc;
	short *sample_data = NULL;

    char *input = NULL;
    char *output = NULL;

	/* Decode input */

	clock_t start_decode = clock();

	if (QOACONV_STR_ENDS_WITH(argv[1], ".wav")) {
		sample_data = qoaconv_wav_read(argv[1], &desc);
	}
	else if (QOACONV_STR_ENDS_WITH(argv[1], ".mp3")) {
		#ifdef QOACONV_HAS_DRMP3
			sample_data = qoaconv_mp3_read(argv[1], &desc);
		#else
			QOACONV_ABORT("qoaconv was not compiled with an MP3 decoder (QOACONV_HAS_DRMP3)");
		#endif
	}
	else if (QOACONV_STR_ENDS_WITH(argv[1], ".flac")) {
		#ifdef QOACONV_HAS_DRFLAC
			sample_data = qoaconv_flac_read(argv[1], &desc);
		#else
			QOACONV_ABORT("qoaconv was not compiled with a FLAC decoder (QOACONV_HAS_DRFLAC)");
		#endif
	}
	else if (QOACONV_STR_ENDS_WITH(argv[1], ".qoa") || QOACONV_STR_ENDS_WITH(argv[1], ".vbq")) {
		sample_data = vbq_read(argv[1], &desc);
	}
	else {
		QOACONV_ABORT("Unknown file type for %s", argv[1]);
	}

	clock_t end_decode = clock();

	QOACONV_ASSERT(sample_data, "Can't load/decode %s", argv[1]);

	printf(
		"%s: channels: %d, samplerate: %d hz, samples per channel: %d, duration: %d sec (took %.3f seconds)\n",
		argv[1], desc.channels, desc.samplerate, desc.samples, desc.samples/desc.samplerate,
		(double)(end_decode - start_decode) / CLOCKS_PER_SEC
	);

    /* Configure qoa encoder if necessary */

    vbq_encoder vbq;
    vbq.min_sr = 1;
    vbq.max_sr = 1023;
    vbq.resample = NULL;
    vbq.quality = 32;
    vbq.min_overshoot = 4;
    vbq.extra_overshoot = 0;
    int use_vbr = 1;
    int i;
    for (i = 2; i < argc; i++) {
        if (QOACONV_STR_ENDS_WITH(argv[i], ".wav") || QOACONV_STR_ENDS_WITH(argv[i], ".qoa") || QOACONV_STR_ENDS_WITH(argv[i], ".vbq")) {
            break;
        }
        if (QOACONV_STR_SAME(argv[i], "-cbr")) {
            use_vbr = 0;
        } else if (QOACONV_STR_SAME(argv[i], "-min_sr")) {
            if (argc >= i + 1) {
                i++;
                vbq.min_sr = (atoi(argv[i]) * 1024) / desc.samplerate;
                vbq.min_sr = vbq.min_sr > 1024 ? 1024 : vbq.min_sr;
                vbq.min_sr = vbq.min_sr < 1 ? 1 : vbq.min_sr;
            } else {
                QOACONV_ABORT("No amount provided for '%s' parameter", argv[i]);
            }
        } else if (QOACONV_STR_SAME(argv[i], "-max_sr")) {
            if (argc >= i + 1) {
                i++;
                vbq.max_sr = (atoi(argv[i]) * 1024) / desc.samplerate;
                vbq.max_sr = vbq.max_sr > 1024 ? 1024 : vbq.max_sr;
                vbq.max_sr = vbq.max_sr < 1 ? 1 : vbq.max_sr;
            } else {
                QOACONV_ABORT("No amount provided for '%s' parameter", argv[i]);
            }
        } else if (QOACONV_STR_SAME(argv[i], "-q")) {
            if (argc >= i + 1) {
                i++;
                vbq.quality = atoi(argv[i]);
            } else {
                QOACONV_ABORT("No amount provided for '%s' parameter", argv[i]);
            }
        } else if (QOACONV_STR_SAME(argv[i], "-min_overshoot")) {
            if (argc >= i + 1) {
                i++;
                vbq.min_overshoot = atoi(argv[i]);
            } else {
                QOACONV_ABORT("No amount provided for '%s' parameter", argv[i]);
            }
        } else if (QOACONV_STR_SAME(argv[i], "-extra_overshoot")) {
            if (argc >= i + 1) {
                i++;
                vbq.extra_overshoot = atoi(argv[i]);
            } else {
                QOACONV_ABORT("No amount provided for '%s' parameter", argv[i]);
            }
        } else if (QOACONV_STR_SAME(argv[i], "-sinc")) {
            vbq.resample = &vbqconv_sinc_resample;
        } else if (QOACONV_STR_SAME(argv[i], "-sinc_zeros")) {
            if (argc >= i + 1) {
                i++;
                sinc_zeros = atoi(argv[i]);
            } else {
                QOACONV_ABORT("No amount provided for '%s' parameter", argv[i]);
            }
        } else if (QOACONV_STR_SAME(argv[i], "-sinc_spz")) {
            if (argc >= i + 1) {
                i++;
                sinc_spz = atoi(argv[i]);
            } else {
                QOACONV_ABORT("No amount provided for '%s' parameter", argv[i]);
            }
        } else {
            QOACONV_ABORT("Unknown parameter %s", argv[i]);
        }
    }

    QOACONV_ASSERT(i < argc, "\nNo valid output provided.\n");

	/* Encode output */
	clock_t start_encode = clock();

	int bytes_written = 0;
	double psnr = INFINITY;
	if (QOACONV_STR_ENDS_WITH(argv[i], ".wav")) {
		bytes_written = qoaconv_wav_write(argv[i], sample_data, &desc);
	}
	else if (QOACONV_STR_ENDS_WITH(argv[i], ".qoa")) {
        bytes_written = vbq_write(argv[i], sample_data, &desc, NULL);
		#ifdef QOA_RECORD_TOTAL_ERROR
			if (desc.error) {
				psnr = -20.0 * log10(sqrt(desc.error/(desc.samples * desc.channels)) / 32768.0);
			}
		#endif
	}
    else if (QOACONV_STR_ENDS_WITH(argv[i], ".vbq")) {
        if (use_vbr) {
            if (vbq.resample) {
                vbqconv_build_sinc_table();
            }
            bytes_written = vbq_write(argv[i], sample_data, &desc, &vbq);
        } else {
            bytes_written = vbq_write(argv[i], sample_data, &desc, NULL);
        }
		#ifdef QOA_RECORD_TOTAL_ERROR
			if (desc.error) {
				psnr = -20.0 * log10(sqrt(desc.error/(desc.samples * desc.channels)) / 32768.0);
			}
		#endif
    }
	else {
		QOACONV_ABORT("Unknown file type for %s", argv[i]);
	}

	clock_t end_encode = clock();

	QOACONV_ASSERT(bytes_written, "Can't write/encode %s", argv[i]);
	free(sample_data);
    if (sinc_table) {
        free(sinc_table);
    }

	printf(
		"%s: size: %d kb (%d bytes) = %.2f kbit/s, psnr: %.2f db (took %.4f seconds)\n",
		argv[i], bytes_written/1024, bytes_written, 
		(((float)bytes_written*8)/((float)desc.samples/(float)desc.samplerate))/1024, psnr,
		(double)(end_encode - start_encode) / CLOCKS_PER_SEC
	);

	return 0;
}
