/*
 * Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef VOICE_AEC_PROCESSOR_H
#define VOICE_AEC_PROCESSOR_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct VoiceAecProcessor VoiceAecProcessor;

VoiceAecProcessor* voice_aec_create(int sample_rate,
                                    int noise_suppression_enabled,
                                    int high_pass_filter_enabled);

void voice_aec_destroy(VoiceAecProcessor* processor);

int voice_aec_frame_size(const VoiceAecProcessor* processor);

int voice_aec_process(VoiceAecProcessor* processor,
                        const int16_t* microphone,
                        const int16_t* playback_reference,
                        size_t frame_count,
                        int16_t* output);

#ifdef __cplusplus
}
#endif

#endif  // VOICE_AEC_PROCESSOR_H
