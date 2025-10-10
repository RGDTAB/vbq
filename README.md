# VBQ - “Variable Bitrate QOA” for Quite Okay Audio at lower file sizes

Single-file MIT licensed library for C/C++

VBQ aims to take advantage of the frequency information within audio
to typically offer lower file sizes. It does this by downsampling frames
in which high frequencies fall below the absolute threshold for human
hearing. At decode time, it linearly upsamples each frame to the
original sample rate.

## Efficiency
The compression efficiency gains vary from sample to sample. In some
situations where high frequencies are frequently audible, just using QOA
downsampled to 36000hz could give better audio quality. vbqconv offers
multiple ways to balance between compression efficiency and audio quality.

## Artifacting
A 'muffling' effect is the most common form of artifacting, but it's
also possible for 'imaging' artifacts to occur during linear interpolation.
The VBQ encoder attempts to combat the latter by reflecting the frequency
spectrum across the new samplerate, to ensure that the reflection falls
under the threshold of human hearing.

## Performance
Encoding takes around 2.0x-3.0x times as long as QOA while using cubic
resampling. When using the sinc resampler, performance will depend
heavily on the window size, with larger window sizes causing encoding
times to skyrocket.  
Decoding times are competitive with stock QOA. This is due mostly to
the fact that linear interpolation is just so cheap to perform.

See [vbq.h](https://github.com/rgdtab/vbq/blob/master/vbq.h) for
a detailed description of the differences in file format specification
between vbq and qoa.

⚠️ VBQ format hasn't been finalized.  
⚠️ This implementation has not yet been fuzzed. Don't use it with untrusted input.

