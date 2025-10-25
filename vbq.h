/*

Copyright (c) 2023, Dominic Szablewski - https://phoboslab.org
Copyright (c) 2025, Raymond DiDonato
SPDX-License-Identifier: MIT

VBQ - A "Variable Bitrate QOA" implementation for fast, lossy audio compression

-- Data Format
VBQ works by dynamically downsampling audio before encoding it QOA. The file
specifications are nearly identical to QOA, but there was a need to create a new
file format due to the following key differences:

- Each frame may have any number of slices, up to and including 256 slices.
- The last slice on any frame may contain less than 20 samples. But as with QOA, they must
  still use the full 8 bytes.
- The number of samples stated in the file header is the number of samples received after
  upsampling.
- The samplerate in any frame may differ from that in the first frame, even in static files.
- The samplerate of any frame may not exceed the samplerate of the first frame.
- Well defined way to handle frames of different samplerates.
The obvious cost of these changes is that VBQ is not easily seekable like QOA.

-- Performance
In most cases, VBQ presents a similar performance cost to QOA. This is likely due to
the fact that downsampling and upsampling is typically cheaper than encoding and decoding
with QOA. As the bitrate of VBQ increases, so will the performance cost to encode and
decode. Eventually, the cost will surpass that of QOA.

-- Artifacting
There are four primary forms of artifacting with VBQ
1. QOA style artifacting, which usually sounds like white noise. This can occur more
   often than in QOA, especially if resampling causes the new samplerate to 'hug' the
   max frequency much closer.
2. Muffling that occurs when audible frequencies get thrown out by mistake.
3. Downsampling errors. The cubic downsampler doesn't completely silence frequencies
   over half the new samplerate, resulting in those frequencies getting 'aliased' into
   the accepted frequencies. It can also muffle frequencies close to half of the new
   samplerate. The sinc resampler provided in vbqconv has a much flatter frequency
   response, so it has much less aliasing and muffling than cubic at a much higher cost.
4. Upsampling errors. Linear upsampling doesn't have a very flat frequency response, so
   higher frequencies can become very muffled. It can also result in imaging errors, but
   the psychoacoustic model has built-in protections to prevent them from being overly
   audible.

-- Resampling
When resampling each frame, the distance between each new sample should be
[original_sr / frame_sr] when encoding, and [frame_sr / original_sr] when decoding,
in front of the previous generated sample. Integer positions represent the input samples, whether
those are the source samples, or the output from the QOA decoder.
For convenience, we'll call this distance a step.

For the very first generated sample, the position should be
[step - 1.0]
When downsampling/encoding, this means that the last sample generated will be the same as the last
sample of the input to the resampler.
When upsampling/decoding, negative values should be treated as interpolation between the
last sample of the previous frame, and the first sample of the current frame that QOA decoded.

So,
ResampleBuffer[0] = SampleData[step - 1.0],
ResampleBuffer[1] = SampleData[2 * step - 1.0],
ResampleBuffer[2] = SampleData[3 * step - 1.0],
...
ResampleBuffer[k - 1] = SampleData[n - 1]
Where n is the size of the input, and k is the size of the output.

By default, VBQ uses cubic resampling for downsampling, as that played the nicest with QOA
during my testing. Linear resampling is performed for upsampling, as
it's the fastest interpolation technique. The downsampler can be overriden, and vbqconv
provides the option to perform slower sinc resampling, which may improve quality by reducing
aliasing of high frequencies.

*/


/* -----------------------------------------------------------------------------
	Header - Public functions */

#ifndef QOA_H
#define QOA_H

#ifdef __cplusplus
extern "C" {
#endif

#define QOA_MIN_FILESIZE 16
#define QOA_MAX_CHANNELS 8

#define QOA_SLICE_LEN 20
#define QOA_SLICES_PER_FRAME 256
#define QOA_FRAME_LEN (QOA_SLICES_PER_FRAME * QOA_SLICE_LEN)
#define QOA_LMS_LEN 4
#define QOA_MAGIC 0x716f6166 /* 'qoaf' */

#define QOA_FRAME_SIZE(channels, slices) \
	(8 + QOA_LMS_LEN * 4 * channels + 8 * slices * channels)

#define VBQ_MAGIC 0x76627166 /* 'vbqf' */

#define VBQ_FFT_LEN 1024
#define VBQ_FFT_HALF 512

#define VBQ_PI 1024
#define VBQ_SINE_MAX 8192

#define VBQ_MIX_SIZE 65536

typedef struct {
	int history[QOA_LMS_LEN];
	int weights[QOA_LMS_LEN];
} qoa_lms_t;

typedef struct {
	unsigned int channels;
	unsigned int samplerate;
	unsigned int samples;
	qoa_lms_t lms[QOA_MAX_CHANNELS];
	#ifdef QOA_RECORD_TOTAL_ERROR
		double error;
	#endif
} qoa_desc;

typedef struct {
    void (*resample)(const short *, qoa_desc *, unsigned int, unsigned int, unsigned int, short *);
    unsigned int minimum_samplerate;
    unsigned int maximum_samplerate;
    unsigned int quality;
    unsigned int min_overshoot;
    int extra_overshoot;
} vbq_encoder;

unsigned int qoa_encode_header(qoa_desc *qoa, unsigned char *bytes);
unsigned int qoa_encode_frame(const short *sample_data, qoa_desc *qoa, unsigned int frame_len, unsigned char *bytes);

unsigned int vbq_encode_header(qoa_desc *qoa, unsigned char *bytes);
void *vbq_encode(const short *sample_data, qoa_desc *qoa, vbq_encoder *vbq, unsigned int *out_len);

unsigned int qoa_max_frame_size(qoa_desc *qoa);
unsigned int vbq_decode_header(const unsigned char *bytes, int size, qoa_desc *qoa);
unsigned int vbq_decode_frame(const unsigned char *bytes, unsigned int size, qoa_desc *qoa, short *sample_data, unsigned int *frame_len);
short *vbq_decode(const unsigned char *bytes, int size, qoa_desc *file);

#ifndef QOA_NO_STDIO

int vbq_write(const char *filename, const short *sample_data, qoa_desc *qoa, vbq_encoder *vbq);
void *vbq_read(const char *filename, qoa_desc *qoa);

#endif /* QOA_NO_STDIO */


#ifdef __cplusplus
}
#endif
#endif /* QOA_H */


/* -----------------------------------------------------------------------------
	Implementation */

#ifdef VBQ_IMPLEMENTATION
#include <stdlib.h>

#ifndef QOA_MALLOC
	#define QOA_MALLOC(sz) malloc(sz)
	#define QOA_FREE(p) free(p)
#endif

typedef unsigned long long qoa_uint64_t;


/* The quant_tab provides an index into the dequant_tab for residuals in the
range of -8 .. 8. It maps this range to just 3bits and becomes less accurate at 
the higher end. Note that the residual zero is identical to the lowest positive 
value. This is mostly fine, since the qoa_div() function always rounds away 
from zero. */

static const int qoa_quant_tab[17] = {
	7, 7, 7, 5, 5, 3, 3, 1, /* -8..-1 */
	0,                      /*  0     */
	0, 2, 2, 4, 4, 6, 6, 6  /*  1.. 8 */
};


/* We have 16 different scalefactors. Like the quantized residuals these become
less accurate at the higher end. In theory, the highest scalefactor that we
would need to encode the highest 16bit residual is (2**16)/8 = 8192. However we
rely on the LMS filter to predict samples accurately enough that a maximum 
residual of one quarter of the 16 bit range is sufficient. I.e. with the 
scalefactor 2048 times the quant range of 8 we can encode residuals up to 2**14.

The scalefactor values are computed as:
scalefactor_tab[s] <- round(pow(s + 1, 2.75)) */

static const int qoa_scalefactor_tab[16] = {
	1, 7, 21, 45, 84, 138, 211, 304, 421, 562, 731, 928, 1157, 1419, 1715, 2048
};


/* The reciprocal_tab maps each of the 16 scalefactors to their rounded 
reciprocals 1/scalefactor. This allows us to calculate the scaled residuals in 
the encoder with just one multiplication instead of an expensive division. We 
do this in .16 fixed point with integers, instead of floats.

The reciprocal_tab is computed as:
reciprocal_tab[s] <- ((1<<16) + scalefactor_tab[s] - 1) / scalefactor_tab[s] */

static const int qoa_reciprocal_tab[16] = {
	65536, 9363, 3121, 1457, 781, 475, 311, 216, 156, 117, 90, 71, 57, 47, 39, 32
};


/* The dequant_tab maps each of the scalefactors and quantized residuals to 
their unscaled & dequantized version.

Since qoa_div rounds away from the zero, the smallest entries are mapped to 3/4
instead of 1. The dequant_tab assumes the following dequantized values for each 
of the quant_tab indices and is computed as:
float dqt[8] = {0.75, -0.75, 2.5, -2.5, 4.5, -4.5, 7, -7};
dequant_tab[s][q] <- round_ties_away_from_zero(scalefactor_tab[s] * dqt[q])

The rounding employed here is "to nearest, ties away from zero",  i.e. positive
and negative values are treated symmetrically.
*/

static const int qoa_dequant_tab[16][8] = {
	{   1,    -1,    3,    -3,    5,    -5,     7,     -7},
	{   5,    -5,   18,   -18,   32,   -32,    49,    -49},
	{  16,   -16,   53,   -53,   95,   -95,   147,   -147},
	{  34,   -34,  113,  -113,  203,  -203,   315,   -315},
	{  63,   -63,  210,  -210,  378,  -378,   588,   -588},
	{ 104,  -104,  345,  -345,  621,  -621,   966,   -966},
	{ 158,  -158,  528,  -528,  950,  -950,  1477,  -1477},
	{ 228,  -228,  760,  -760, 1368, -1368,  2128,  -2128},
	{ 316,  -316, 1053, -1053, 1895, -1895,  2947,  -2947},
	{ 422,  -422, 1405, -1405, 2529, -2529,  3934,  -3934},
	{ 548,  -548, 1828, -1828, 3290, -3290,  5117,  -5117},
	{ 696,  -696, 2320, -2320, 4176, -4176,  6496,  -6496},
	{ 868,  -868, 2893, -2893, 5207, -5207,  8099,  -8099},
	{1064, -1064, 3548, -3548, 6386, -6386,  9933,  -9933},
	{1286, -1286, 4288, -4288, 7718, -7718, 12005, -12005},
	{1536, -1536, 5120, -5120, 9216, -9216, 14336, -14336},
};


/* The Least Mean Squares Filter is the heart of QOA. It predicts the next
sample based on the previous 4 reconstructed samples. It does so by continuously
adjusting 4 weights based on the residual of the previous prediction.

The next sample is predicted as the sum of (weight[i] * history[i]).

The adjustment of the weights is done with a "Sign-Sign-LMS" that adds or
subtracts the residual to each weight, based on the corresponding sample from 
the history. This, surprisingly, is sufficient to get worthwhile predictions.

This is all done with fixed point integers. Hence the right-shifts when updating
the weights and calculating the prediction. */

static int qoa_lms_predict(qoa_lms_t *lms) {
	int prediction = 0;
	for (int i = 0; i < QOA_LMS_LEN; i++) {
		prediction += lms->weights[i] * lms->history[i];
	}
	return prediction >> 13;
}

static void qoa_lms_update(qoa_lms_t *lms, int sample, int residual) {
	int delta = residual >> 4;
	for (int i = 0; i < QOA_LMS_LEN; i++) {
		lms->weights[i] += lms->history[i] < 0 ? -delta : delta;
	}

	for (int i = 0; i < QOA_LMS_LEN-1; i++) {
		lms->history[i] = lms->history[i+1];
	}
	lms->history[QOA_LMS_LEN-1] = sample;
}


/* qoa_div() implements a rounding division, but avoids rounding to zero for 
small numbers. E.g. 0.1 will be rounded to 1. Note that 0 itself still 
returns as 0, which is handled in the qoa_quant_tab[].
qoa_div() takes an index into the .16 fixed point qoa_reciprocal_tab as an
argument, so it can do the division with a cheaper integer multiplication. */

static inline int qoa_div(int v, int scalefactor) {
	int reciprocal = qoa_reciprocal_tab[scalefactor];
	int n = (v * reciprocal + (1 << 15)) >> 16;
	n = n + ((v > 0) - (v < 0)) - ((n > 0) - (n < 0)); /* round away from 0 */
	return n;
}

static inline int qoa_clamp(int v, int min, int max) {
	if (v < min) { return min; }
	if (v > max) { return max; }
	return v;
}

/* This specialized clamp function for the signed 16 bit range improves decode
performance quite a bit. The extra if() statement works nicely with the CPUs
branch prediction as this branch is rarely taken. */

static inline int qoa_clamp_s16(int v) {
	if ((unsigned int)(v + 32768) > 65535) {
		if (v < -32768) { return -32768; }
		if (v >  32767) { return  32767; }
	}
	return v;
}

static inline qoa_uint64_t qoa_read_u64(const unsigned char *bytes, unsigned int *p) {
	bytes += *p;
	*p += 8;
	return 
		((qoa_uint64_t)(bytes[0]) << 56) | ((qoa_uint64_t)(bytes[1]) << 48) |
		((qoa_uint64_t)(bytes[2]) << 40) | ((qoa_uint64_t)(bytes[3]) << 32) |
		((qoa_uint64_t)(bytes[4]) << 24) | ((qoa_uint64_t)(bytes[5]) << 16) |
		((qoa_uint64_t)(bytes[6]) <<  8) | ((qoa_uint64_t)(bytes[7]) <<  0);
}

static inline void qoa_write_u64(qoa_uint64_t v, unsigned char *bytes, unsigned int *p) {
	bytes += *p;
	*p += 8;
	bytes[0] = (v >> 56) & 0xff;
	bytes[1] = (v >> 48) & 0xff;
	bytes[2] = (v >> 40) & 0xff;
	bytes[3] = (v >> 32) & 0xff;
	bytes[4] = (v >> 24) & 0xff;
	bytes[5] = (v >> 16) & 0xff;
	bytes[6] = (v >>  8) & 0xff;
	bytes[7] = (v >>  0) & 0xff;
}


/* -----------------------------------------------------------------------------
	Encoder */

unsigned int qoa_encode_header(qoa_desc *qoa, unsigned char *bytes) {
	unsigned int p = 0;
	qoa_write_u64(((qoa_uint64_t)QOA_MAGIC << 32) | qoa->samples, bytes, &p);
	return p;
}

unsigned int vbq_encode_header(qoa_desc *qoa, unsigned char *bytes) {
	unsigned int p = 0;
	qoa_write_u64(((qoa_uint64_t)VBQ_MAGIC << 32) | qoa->samples, bytes, &p);
	return p;
}

/** Generated using Dr LUT - Free Lookup Table Generator
  * https://github.com/ppelikan/drlut
  **/
// Formula: sin(2*pi*t/T) 
const short vbq_sin_table[256] = {
     0,   201,   402,   603,   803,  1003,  1202,
  1401,  1598,  1795,  1990,  2185,  2378,  2570,
  2760,  2948,  3135,  3320,  3503,  3683,  3862,
  4038,  4212,  4383,  4551,  4717,  4880,  5040,
  5197,  5351,  5501,  5649,  5793,  5933,  6070,
  6203,  6333,  6458,  6580,  6698,  6811,  6921,
  7027,  7128,  7225,  7317,  7405,  7489,  7568,
  7643,  7713,  7779,  7839,  7895,  7946,  7993,
  8035,  8071,  8103,  8130,  8153,  8170,  8182,
  8190,  8192,  8190,  8182,  8170,  8153,  8130,
  8103,  8071,  8035,  7993,  7946,  7895,  7839,
  7779,  7713,  7643,  7568,  7489,  7405,  7317,
  7225,  7128,  7027,  6921,  6811,  6698,  6580,
  6458,  6333,  6203,  6070,  5933,  5793,  5649,
  5501,  5351,  5197,  5040,  4880,  4717,  4551,
  4383,  4212,  4038,  3862,  3683,  3503,  3320,
  3135,  2948,  2760,  2570,  2378,  2185,  1990,
  1795,  1598,  1401,  1202,  1003,   803,   603,
   402,   201,     0,  -201,  -402,  -603,  -803,
 -1003, -1202, -1401, -1598, -1795, -1990, -2185,
 -2378, -2570, -2760, -2948, -3135, -3320, -3503,
 -3683, -3862, -4038, -4212, -4383, -4551, -4717,
 -4880, -5040, -5197, -5351, -5501, -5649, -5793,
 -5933, -6070, -6203, -6333, -6458, -6580, -6698,
 -6811, -6921, -7027, -7128, -7225, -7317, -7405,
 -7489, -7568, -7643, -7713, -7779, -7839, -7895,
 -7946, -7993, -8035, -8071, -8103, -8130, -8153,
 -8170, -8182, -8190, -8192, -8190, -8182, -8170,
 -8153, -8130, -8103, -8071, -8035, -7993, -7946,
 -7895, -7839, -7779, -7713, -7643, -7568, -7489,
 -7405, -7317, -7225, -7128, -7027, -6921, -6811,
 -6698, -6580, -6458, -6333, -6203, -6070, -5933,
 -5793, -5649, -5501, -5351, -5197, -5040, -4880,
 -4717, -4551, -4383, -4212, -4038, -3862, -3683,
 -3503, -3320, -3135, -2948, -2760, -2570, -2378,
 -2185, -1990, -1795, -1598, -1401, -1202, -1003,
  -803,  -603,  -402,  -201 };

static inline int vbq_sin(long angle) {
    const int LUT_PI = 128;
    angle = (2 * VBQ_PI + angle % (2 * VBQ_PI)) % (2 * VBQ_PI);
    int index = angle / (VBQ_PI / LUT_PI);
    int mix = angle % (VBQ_PI / LUT_PI);
    int s1 = vbq_sin_table[index];
    int s2 = vbq_sin_table[(index + 1) % (2 * LUT_PI)];
    return s1 + mix * (s2 - s1) / (VBQ_PI / LUT_PI);
}

static inline int vbq_cos(long angle) {
    return vbq_sin(angle + VBQ_PI / 2);
}

static inline long vbq_sqrt(unsigned long long val, unsigned long guess) {
    unsigned long long b;
    if (val < 2) {
        return val;
    }
    b = val / guess; guess = (guess + b) / 2;
    b = val / guess; guess = (guess + b) / 2;
    b = val / guess; guess = (guess + b) / 2;
    return guess;
}

const unsigned char BitReverseTable256[] = 
{
  0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0, 
  0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 
  0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4, 
  0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC, 
  0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2, 
  0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
  0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6, 
  0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
  0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
  0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9, 
  0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
  0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
  0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3, 
  0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
  0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7, 
  0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};

static void vbq_fft(const short *sample_data, unsigned int channels, unsigned int channel, unsigned long *out) {
    long *real = out;
    long imag[VBQ_FFT_LEN] = {0};
    const unsigned int fft_len = VBQ_FFT_LEN;
    const unsigned int shift_len = 6;

    for (unsigned int i = 0; i < fft_len; i++) {
        unsigned char b1 = i & 0xFF;
        unsigned char b2 = (i >> 8) & 0xFF;
        b1 = BitReverseTable256[b1];
        b2 = BitReverseTable256[b2];

        unsigned int k = ((b1 << 8) | b2) >> shift_len;
        if (k > i) {
            int sink = vbq_sin(VBQ_PI * k / fft_len);
            sink = (sink * sink) / VBQ_SINE_MAX;
            int sini = vbq_sin(VBQ_PI * i / fft_len);
            sini = (sini * sini) / VBQ_SINE_MAX;

            real[i] = (sink * sample_data[k * channels + channel]) / VBQ_SINE_MAX;
            real[k] = (sini * sample_data[i * channels + channel]) / VBQ_SINE_MAX;

        } else if (k == i) {
            int sini = vbq_sin(VBQ_PI * i / fft_len);
            sini = (sini * sini) / VBQ_SINE_MAX;

            real[i] = (sini * sample_data[i * channels + channel]) / VBQ_SINE_MAX;
        }
    }

    // Cooley-Tukey, taken from wikipedia and modified for numerical stability
    for (unsigned int s = 1; s < fft_len; s <<= 1) {
        unsigned int m = s * 2;
        int angle_m = (-2 * VBQ_PI) / m;

        for (unsigned int k = 0; k < fft_len; k += m) {
            int angle = 0;
            for (unsigned int j = 0; j < s; j++) {
                int w_r = vbq_cos(angle);
                int w_i = vbq_sin(angle);
                unsigned int p_1 = k + j + s;
                unsigned int p_2 = k + j;

                long t_r = (w_r * real[p_1] - w_i * imag[p_1]) / VBQ_SINE_MAX;
                long t_i = (w_r * imag[p_1] + w_i * real[p_1]) / VBQ_SINE_MAX;

                real[p_1] = real[p_2] - t_r;
                imag[p_1] = imag[p_2] - t_i;
                real[p_2] += t_r;
                imag[p_2] += t_i;

                angle += angle_m;
            }
        }
    }

    for (unsigned int i = 0; i < VBQ_FFT_HALF; i++) {
        unsigned long ar = real[i] < 0 ? -real[i] : real[i];
        unsigned long ai = imag[i] < 0 ? -imag[i] : imag[i];
        unsigned long guess = ar > ai ? ar : ai;
        unsigned long long mag_sqr = real[i] * real[i] + imag[i] * imag[i];
        out[i] = vbq_sqrt(mag_sqr, guess) / 32;
    }
}

static unsigned int vbq_psychoacoustic_model(long *magnitudes, unsigned int samplerate, vbq_encoder *vbq) {
    /* Created from ISO 226:2023, using the formula 10^(db/20) to convert from decibels to multipliers */
    static const long thresh_mults[] = {
        54,
        46,    // 1khz
        28,    // 2khz
        16,    // 3khz
        17,    // 4khz
        27,    // 5khz
        55,    // 6khz
        100,    // 7khz
        143,   // 8khz
        161,   // 9khz
        154,   // 10khz
        138,   // 11khz
        124,   // 12khz
        121,   // 13khz
        136,   // 14khz
        191,   // 15khz
        358,   // 16khz
        956,   // 17khz
        3909,   // 18khz
        26249,  // 19khz
        310466,  // 20khz
        6936652,
    };
    /* Frequency response of linear interpolation, sinc^2 */
    static const int mirror_mults[] = {
        415,
        364,
        315,
        270,
        227,
        187,
        152,
        120,
        92,
        68,
        49,
        32,
        20,
        11,
        4,
        1,
        0,
    };


    int multiplier;
    int max = (20000 * VBQ_FFT_LEN) / samplerate;
    max = max > VBQ_FFT_HALF - 1 ? VBQ_FFT_HALF - 1 : max;
    int highest = max;
    while (highest) {
        int freq = (highest * samplerate) / 1000;
        int lut_pos = freq / VBQ_FFT_LEN;
        int mix = freq % VBQ_FFT_LEN;
        multiplier = thresh_mults[lut_pos];
        multiplier += (mix * (thresh_mults[lut_pos + 1] - multiplier)) / VBQ_FFT_LEN;

        int threshold = (multiplier * vbq->quality) / 16;
        if (magnitudes[highest] > threshold) {
            break;
        }
        highest--;
    }

    int i = 1;
    highest += vbq->min_overshoot;
    while (i < highest && highest + i < VBQ_FFT_HALF) {
        int mirror = highest + i;
        int freq = (mirror * samplerate) / 1000;
        int lut_pos = freq / VBQ_FFT_LEN;
        int mix = freq % VBQ_FFT_LEN;
        multiplier = thresh_mults[lut_pos];
        multiplier += (mix * (thresh_mults[lut_pos + 1] - multiplier)) / VBQ_FFT_LEN;

        int threshold = (multiplier * vbq->quality) / 16;

        lut_pos = i / 16;
        mix = i % 16;
        int mirror_scale = mirror_mults[lut_pos];
        mirror_scale += (mix * (mirror_mults[lut_pos + 1] - mirror_scale)) / 16;
        if ((magnitudes[highest - i + 1] * mirror_scale) / 1024 > threshold) {
            highest++;
            i = 0;
        }
        i++;
    }
    highest += vbq->extra_overshoot;

    return highest * 2;
}

unsigned int vbq_minimum_samplerate(const short *sample_data, qoa_desc *qoa, vbq_encoder *vbq) {
    unsigned int highest_samplerate = qoa_clamp(vbq->minimum_samplerate, 1, VBQ_FFT_LEN);

    long magnitudes[VBQ_FFT_LEN];

    for (unsigned int i = 0; i < qoa->channels; i++) {
        vbq_fft(sample_data, qoa->channels, i, magnitudes);
        unsigned int channel_samplerate = vbq_psychoacoustic_model(magnitudes, qoa->samplerate, vbq);
        highest_samplerate = channel_samplerate > highest_samplerate ? channel_samplerate : highest_samplerate;
    }

    unsigned int max = qoa_clamp(vbq->maximum_samplerate, 1, VBQ_FFT_LEN);
    highest_samplerate = highest_samplerate > max ? max : highest_samplerate;
    return highest_samplerate;
}

void vbq_cubic_resample(const short *sample_data, qoa_desc *qoa, unsigned int vbr_len, unsigned int max_ratio, unsigned int sample_index, short *resample_buffer) {
    const long MIX_SIZE = VBQ_MIX_SIZE;
    unsigned long step = (MIX_SIZE * VBQ_FFT_LEN) / max_ratio;

    for (int c = 0; c < qoa->channels; c++) {
        long offset = step - MIX_SIZE;
        for (unsigned int s = c; s < vbr_len * max_ratio * qoa->channels; s += qoa->channels) {
            long pos = (offset / MIX_SIZE);
            long mix = offset % MIX_SIZE;
            int ym1;
            int y0 = sample_data[pos * qoa->channels + c];
            int y1;
            int y2;
            if (sample_index + pos < 1) {
                ym1 = sample_data[pos * qoa->channels + c];
            } else {
                ym1 = sample_data[(pos - 1) * qoa->channels + c];
            }
            if (qoa->samples - sample_index - pos < 2) {
                y1 = sample_data[pos * qoa->channels + c];
            } else {
                y1 = sample_data[(pos + 1) * qoa->channels + c];
            }
            if (qoa->samples - sample_index - pos < 3) {
                y2 = sample_data[pos * qoa->channels + c];
            } else {
                y2 = sample_data[(pos + 2) * qoa->channels + c];
            }

            long c0 = y0;
            long c1 = (y1 - ym1) / 2;
            long c2 = ym1 - (5 * y0) / 2 + 2 * y1 - y2 / 2;
            long c3 = (y2 - ym1) / 2 + (3 * (y0 - y1)) / 2;

            int out = (c3 * mix) / MIX_SIZE + c2;
            out = (out * mix) / MIX_SIZE + c1;
            out = (out * mix) / MIX_SIZE + c0;

            resample_buffer[s] = qoa_clamp_s16(out);

            offset += step;
        }
    }
}

unsigned int qoa_encode_frame(const short *sample_data, qoa_desc *qoa, unsigned int frame_len, unsigned char *bytes) {
	unsigned int channels = qoa->channels;

	unsigned int p = 0;
	unsigned int slices = (frame_len + QOA_SLICE_LEN - 1) / QOA_SLICE_LEN;
	unsigned int frame_size = QOA_FRAME_SIZE(channels, slices);
	int prev_scalefactor[QOA_MAX_CHANNELS] = {0};

	/* Write the frame header */
    qoa_write_u64((
        (qoa_uint64_t)qoa->channels   << 56 |
        (qoa_uint64_t)qoa->samplerate << 32 |
        (qoa_uint64_t)frame_len       << 16 |
        (qoa_uint64_t)frame_size
    ), bytes, &p);

    for (unsigned int c = 0; c < channels; c++) {
        /* Write the current LMS state */
        qoa_uint64_t weights = 0;
        qoa_uint64_t history = 0;
        for (int i = 0; i < QOA_LMS_LEN; i++) {
            history = (history << 16) | (qoa->lms[c].history[i] & 0xffff);
            weights = (weights << 16) | (qoa->lms[c].weights[i] & 0xffff);
        }
        qoa_write_u64(history, bytes, &p);
        qoa_write_u64(weights, bytes, &p);
    }

	/* We encode all samples with the channels interleaved on a slice level.
	E.g. for stereo: (ch-0, slice 0), (ch 1, slice 0), (ch 0, slice 1), ...*/
	for (unsigned int sample_index = 0; sample_index < frame_len; sample_index += QOA_SLICE_LEN) {

		for (unsigned int c = 0; c < channels; c++) {
			int slice_len = qoa_clamp(QOA_SLICE_LEN, 0, frame_len - sample_index);
			int slice_start = sample_index * channels + c;
			int slice_end = (sample_index + slice_len) * channels + c;			

			/* Brute for search for the best scalefactor. Just go through all
			16 scalefactors, encode all samples for the current slice and 
			meassure the total squared error. */
			qoa_uint64_t best_rank = -1;
			#ifdef QOA_RECORD_TOTAL_ERROR
				qoa_uint64_t best_error = -1;
			#endif
			qoa_uint64_t best_slice = 0;
			qoa_lms_t best_lms;
			int best_scalefactor = 0;

			for (int sfi = 0; sfi < 16; sfi++) {
				/* There is a strong correlation between the scalefactors of
				neighboring slices. As an optimization, start testing
				the best scalefactor of the previous slice first. */
				int scalefactor = (sfi + prev_scalefactor[c]) % 16;

				/* We have to reset the LMS state to the last known good one
				before trying each scalefactor, as each pass updates the LMS
				state when encoding. */
				qoa_lms_t lms = qoa->lms[c];
				qoa_uint64_t slice = scalefactor;
				qoa_uint64_t current_rank = 0;
				#ifdef QOA_RECORD_TOTAL_ERROR
					qoa_uint64_t current_error = 0;
				#endif

				for (int si = slice_start; si < slice_end; si += channels) {
					int sample = sample_data[si];
					int predicted = qoa_lms_predict(&lms);

					int residual = sample - predicted;
					int scaled = qoa_div(residual, scalefactor);
					int clamped = qoa_clamp(scaled, -8, 8);
					int quantized = qoa_quant_tab[clamped + 8];
					int dequantized = qoa_dequant_tab[scalefactor][quantized];
					int reconstructed = qoa_clamp_s16(predicted + dequantized);


					/* If the weights have grown too large, we introduce a penalty
					here. This prevents pops/clicks in certain problem cases */
					int weights_penalty = ((
						lms.weights[0] * lms.weights[0] + 
						lms.weights[1] * lms.weights[1] + 
						lms.weights[2] * lms.weights[2] + 
						lms.weights[3] * lms.weights[3]
					) >> 18) - 0x8ff;
					if (weights_penalty < 0) {
						weights_penalty = 0;
					}

					long long error = (sample - reconstructed);
					qoa_uint64_t error_sq = error * error;

					current_rank += error_sq + weights_penalty * weights_penalty;
					#ifdef QOA_RECORD_TOTAL_ERROR
						current_error += error_sq;
					#endif
					if (current_rank > best_rank) {
						break;
					}

					qoa_lms_update(&lms, reconstructed, dequantized);
					slice = (slice << 3) | quantized;
				}

				if (current_rank < best_rank) {
					best_rank = current_rank;
					#ifdef QOA_RECORD_TOTAL_ERROR
						best_error = current_error;
					#endif
					best_slice = slice;
					best_lms = lms;
					best_scalefactor = scalefactor;
				}
			}

			prev_scalefactor[c] = best_scalefactor;

			qoa->lms[c] = best_lms;
			#ifdef QOA_RECORD_TOTAL_ERROR
                qoa->error += best_error;
			#endif

			/* If this slice was shorter than QOA_SLICE_LEN, we have to left-
			shift all encoded data, to ensure the rightmost bits are the empty
			ones.*/
			best_slice <<= (QOA_SLICE_LEN - slice_len) * 3;
            qoa_write_u64(best_slice, bytes, &p);
		}
	}
	
	return p;
}

void *vbq_encode(const short *sample_data, qoa_desc *qoa, vbq_encoder *vbq, unsigned int *out_len) {
	if (
		qoa->samples == 0 || 
		qoa->samplerate <= VBQ_FFT_LEN || qoa->samplerate > 0xffffff ||
		qoa->channels == 0 || qoa->channels > QOA_MAX_CHANNELS
	) {
		return NULL;
	}

	/* Calculate the encoded size and allocate */
	unsigned int num_frames = (qoa->samples + QOA_FRAME_LEN-1) / QOA_FRAME_LEN;
	unsigned int num_slices = (qoa->samples + QOA_SLICE_LEN-1) / QOA_SLICE_LEN;
	unsigned int encoded_size = 8 +                    /* 8 byte file header */
		num_frames * 8 +                               /* 8 byte frame headers */
		num_frames * QOA_LMS_LEN * 4 * qoa->channels + /* 4 * 4 bytes lms state per channel */
		num_slices * 8 * qoa->channels;                /* 8 byte slices */

	unsigned char *bytes = QOA_MALLOC(encoded_size);

	for (unsigned int c = 0; c < qoa->channels; c++) {
		/* Set the initial LMS weights to {0, 0, -1, 2}. This helps with the 
		prediction of the first few ms of a file. */
		qoa->lms[c].weights[0] = 0;
		qoa->lms[c].weights[1] = 0;
		qoa->lms[c].weights[2] = -(1<<13);
		qoa->lms[c].weights[3] =  (1<<14);

		/* Explicitly set the history samples to 0, as we might have some
		garbage in there. */
		for (int i = 0; i < QOA_LMS_LEN; i++) {
			qoa->lms[c].history[i] = 0;
		}
	}

    unsigned int p;
    if (vbq) {
        if (!vbq->resample) {
            vbq->resample = &vbq_cubic_resample;
        }
        if (!vbq->maximum_samplerate) {
            vbq->maximum_samplerate = VBQ_FFT_LEN;
        }
	    p = vbq_encode_header(qoa, bytes);
        qoa_write_u64((
            (qoa_uint64_t)qoa->channels   << 56 |
            (qoa_uint64_t)qoa->samplerate << 32
        ), bytes, &p);
    } else {
	    p = qoa_encode_header(qoa, bytes);
    }
	/* Encode the header and go through all frames */
	#ifdef QOA_RECORD_TOTAL_ERROR
		qoa->error = 0;
	#endif


    unsigned int frame_len;
    unsigned int ratios[QOA_FRAME_LEN / VBQ_FFT_LEN];
    short resample_buffer[(QOA_FRAME_LEN + QOA_LMS_LEN) * QOA_MAX_CHANNELS];
    unsigned int prev_ratio = 0;
	for (unsigned int sample_index = 0; sample_index < qoa->samples; sample_index += frame_len) {
		frame_len = qoa_clamp(QOA_FRAME_LEN, 0, qoa->samples - sample_index);
		const short *frame_samples = sample_data + sample_index * qoa->channels;
        unsigned int frame_size;

        if (vbq && frame_len > VBQ_FFT_LEN) {
            unsigned int base_samplerate = qoa->samplerate;
            unsigned int max_ratio = 0;
            unsigned int min_ratio = 0;
            unsigned int vbr_len;
            if (!prev_ratio) {
                ratios[0] = vbq_minimum_samplerate(frame_samples, qoa, vbq);
            } else {
                ratios[0] = prev_ratio;
            }
            prev_ratio = 0;
            for (vbr_len = 1; vbr_len < (frame_len / VBQ_FFT_LEN); vbr_len++) {
                ratios[vbr_len] = vbq_minimum_samplerate(frame_samples + vbr_len * VBQ_FFT_LEN * qoa->channels, qoa, vbq);
                if (ratios[vbr_len] > ratios[max_ratio]) {
                    if ((ratios[vbr_len] - ratios[min_ratio]) * qoa->channels > ((1 + 2 * qoa->channels) * QOA_SLICE_LEN)) {
                        prev_ratio = ratios[vbr_len];
                        break;
                    }
                    max_ratio = vbr_len;
                } else if (ratios[vbr_len] < ratios[min_ratio]) {
                    if ((ratios[max_ratio] - ratios[vbr_len]) * qoa->channels > ((1 + 2 * qoa->channels) * QOA_SLICE_LEN)) {
                        prev_ratio = ratios[vbr_len];
                        break;
                    }
                    min_ratio = vbr_len;
                }
            }
            max_ratio = ratios[max_ratio];

            if (max_ratio < VBQ_FFT_LEN) {
                vbq->resample(frame_samples, qoa, vbr_len, max_ratio, sample_index, resample_buffer);
                qoa->samplerate = max_ratio;
                frame_size = qoa_encode_frame(resample_buffer, qoa, vbr_len * max_ratio, bytes + p);
                qoa->samplerate = base_samplerate;
            } else {
                frame_size = qoa_encode_frame(frame_samples, qoa, vbr_len * VBQ_FFT_LEN, bytes + p);
            }

            frame_len = vbr_len * VBQ_FFT_LEN;
        } else {
            frame_size = qoa_encode_frame(frame_samples, qoa, frame_len, bytes + p);
        }
		p += frame_size;
	}

	*out_len = p;
	return bytes;
}


/* -----------------------------------------------------------------------------
	Decoder */

unsigned int qoa_max_frame_size(qoa_desc *qoa) {
	return QOA_FRAME_SIZE(qoa->channels, QOA_SLICES_PER_FRAME);
}

unsigned int vbq_decode_header(const unsigned char *bytes, int size, qoa_desc *qoa) {
	unsigned int p = 0;
	if (size < QOA_MIN_FILESIZE) {
		return 0;
	}


	/* Read the file header, verify the magic number ('qoaf') and read the 
	total number of samples. */
	qoa_uint64_t file_header = qoa_read_u64(bytes, &p);

	if ((file_header >> 32) != QOA_MAGIC && (file_header >> 32) != VBQ_MAGIC) {
		return 0;
	}

	qoa->samples = file_header & 0xffffffff;
	if (!qoa->samples) {
		return 0;
	}

	/* Peek into the first frame header to get the number of channels and
	the samplerate. */
	qoa_uint64_t frame_header = qoa_read_u64(bytes, &p);
	qoa->channels   = (frame_header >> 56) & 0x0000ff;
	qoa->samplerate = (frame_header >> 32) & 0xffffff;

	if (qoa->channels == 0 || qoa->samples == 0 || qoa->samplerate == 0) {
		return 0;
	}

    if ((file_header >> 32) == QOA_MAGIC) {
        return 8;
    } else {
        return 16;
    }
}

unsigned int vbq_decode_frame(const unsigned char *bytes, unsigned int size, qoa_desc *qoa, short *sample_data, unsigned int *frame_len) {
	unsigned int p = 0;
	*frame_len = 0;

	if (size < 8 + QOA_LMS_LEN * 4 * qoa->channels) {
		return 0;
	}

	/* Read and verify the frame header */
	qoa_uint64_t frame_header = qoa_read_u64(bytes, &p);
	unsigned int channels   = (frame_header >> 56) & 0x0000ff;
	unsigned int samplerate = (frame_header >> 32) & 0xffffff;
	unsigned int samples    = (frame_header >> 16) & 0x00ffff;
	unsigned int frame_size = (frame_header      ) & 0x00ffff;

	unsigned int data_size = frame_size - 8 - QOA_LMS_LEN * 4 * channels;
	unsigned int num_slices = data_size / 8;
	unsigned int max_total_samples = num_slices * QOA_SLICE_LEN;

	if (
		channels != qoa->channels || 
		frame_size > size ||
		samples * channels > max_total_samples
	) {
		return 0;
	}


	/* Read the LMS state: 4 x 2 bytes history, 4 x 2 bytes weights per channel */
    short previous_samples[QOA_MAX_CHANNELS];
	for (unsigned int c = 0; c < channels; c++) {
		qoa_uint64_t history = qoa_read_u64(bytes, &p);
		qoa_uint64_t weights = qoa_read_u64(bytes, &p);
		for (int i = 0; i < QOA_LMS_LEN; i++) {
			qoa->lms[c].history[i] = ((signed short)(history >> 48));
			history <<= 16;
			qoa->lms[c].weights[i] = ((signed short)(weights >> 48));
			weights <<= 16;
		}
        previous_samples[c] = qoa->lms[c].history[QOA_LMS_LEN - 1];
	}

	/* Decode all slices for all channels in this frame */
    if (samplerate < VBQ_FFT_LEN) {
        const int MIX_SIZE = VBQ_MIX_SIZE;
        unsigned int step = (MIX_SIZE * samplerate) / VBQ_FFT_LEN;
        unsigned int offset = step;
        unsigned int s = 0;

        for (unsigned int sample_index = 0; sample_index < samples; sample_index += QOA_SLICE_LEN) {
            unsigned int slice_start = s;
            unsigned int slice_offset = offset;
            for (unsigned int c = 0; c < channels; c++) {
                s = slice_start + c;
                offset = slice_offset;
                qoa_uint64_t slice = qoa_read_u64(bytes, &p);

                int scalefactor = (slice >> 60) & 0xf;
                slice <<= 4;

                short resample_buffer[QOA_SLICE_LEN];
                int slice_end = qoa_clamp(samples - sample_index, 0, QOA_SLICE_LEN);
                for (int si = 0; si < slice_end; si++) {
                    int predicted = qoa_lms_predict(&qoa->lms[c]);
                    int quantized = (slice >> 61) & 0x7;
                    int dequantized = qoa_dequant_tab[scalefactor][quantized];
                    int reconstructed = qoa_clamp_s16(predicted + dequantized);

                    resample_buffer[si] = reconstructed;
                    slice <<= 3;

                    qoa_lms_update(&qoa->lms[c], reconstructed, dequantized);
                }

                do {
                    int mix = offset;
                    sample_data[s] = previous_samples[c];
                    sample_data[s] += (mix * (resample_buffer[0] - sample_data[s])) / MIX_SIZE;
                    offset += step;
                    s += channels;
                } while (offset / MIX_SIZE < 1);
                offset -= MIX_SIZE;
                int pos = 0;
                while (pos < slice_end - 1) {
                    int mix = offset % MIX_SIZE;
                    sample_data[s] = resample_buffer[pos];
                    sample_data[s] += (mix * (resample_buffer[pos + 1] - sample_data[s])) / MIX_SIZE;

                    offset += step;
                    s += channels;
                    pos = offset / MIX_SIZE;
                }

                previous_samples[c] = qoa->lms[c].history[QOA_LMS_LEN - 1];
                offset %= MIX_SIZE;

                s -= c;
            }
        }

        for (int c = 0; c < channels; c++) {
            sample_data[s + c] = qoa->lms[c].history[QOA_LMS_LEN - 1];
        }
        samples = (samples * VBQ_FFT_LEN) / samplerate;
    } else {
        for (unsigned int sample_index = 0; sample_index < samples; sample_index += QOA_SLICE_LEN) {
            for (unsigned int c = 0; c < channels; c++) {
                qoa_uint64_t slice = qoa_read_u64(bytes, &p);

                int scalefactor = (slice >> 60) & 0xf;
                slice <<= 4;

                int slice_start = sample_index * channels + c;
                int slice_end = qoa_clamp(sample_index + QOA_SLICE_LEN, 0, samples) * channels + c;

                for (int si = slice_start; si < slice_end; si += channels) {
                    int predicted = qoa_lms_predict(&qoa->lms[c]);
                    int quantized = (slice >> 61) & 0x7;
                    int dequantized = qoa_dequant_tab[scalefactor][quantized];
                    int reconstructed = qoa_clamp_s16(predicted + dequantized);
                    
                    sample_data[si] = reconstructed;
                    slice <<= 3;

                    qoa_lms_update(&qoa->lms[c], reconstructed, dequantized);
                }
            }
        }
    }

	*frame_len = samples;
	return p;
}

short *vbq_decode(const unsigned char *bytes, int size, qoa_desc *qoa) {
	unsigned int p = vbq_decode_header(bytes, size, qoa);
	if (!p) {
		return NULL;
	}

	/* Calculate the required size of the sample buffer and allocate */
	int total_samples = qoa->samples * qoa->channels;
	short *sample_data = QOA_MALLOC(total_samples * sizeof(short));

	unsigned int sample_index = 0;
	unsigned int frame_len;
	unsigned int frame_size;

	/* Decode all frames */
	do {
		short *sample_ptr = sample_data + sample_index * qoa->channels;
		frame_size = vbq_decode_frame(bytes + p, size - p, qoa, sample_ptr, &frame_len);

		p += frame_size;
		sample_index += frame_len;
	} while (frame_size && sample_index < qoa->samples);

	qoa->samples = sample_index;
	return sample_data;
}



/* -----------------------------------------------------------------------------
	File read/write convenience functions */

#ifndef QOA_NO_STDIO
#include <stdio.h>

int vbq_write(const char *filename, const short *sample_data, qoa_desc *qoa, vbq_encoder *vbq) {
	FILE *f = fopen(filename, "wb");
	unsigned int size;
	void *encoded;

	if (!f) {
		return 0;
	}

	encoded = vbq_encode(sample_data, qoa, vbq, &size);
	if (!encoded) {
		fclose(f);
		return 0;
	}

	fwrite(encoded, 1, size, f);
	fclose(f);

	QOA_FREE(encoded);
	return size;
}

void *vbq_read(const char *filename, qoa_desc *qoa) {
	FILE *f = fopen(filename, "rb");
	int size, bytes_read;
	void *data;
	short *sample_data;

	if (!f) {
		return NULL;
	}

	fseek(f, 0, SEEK_END);
	size = ftell(f);
	if (size <= 0) {
		fclose(f);
		return NULL;
	}
	fseek(f, 0, SEEK_SET);

	data = QOA_MALLOC(size);
	if (!data) {
		fclose(f);
		return NULL;
	}

	bytes_read = fread(data, 1, size, f);
	fclose(f);

	sample_data = vbq_decode(data, bytes_read, qoa);
	QOA_FREE(data);
	return sample_data;
}

#endif /* QOA_NO_STDIO */
#endif /* VBQ_IMPLEMENTATION */
