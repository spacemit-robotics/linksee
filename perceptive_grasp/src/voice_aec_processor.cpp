/*
 * Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "voice_aec_processor.h"

#include <webrtc/modules/audio_processing/include/audio_processing.h>

#include <algorithm>
#include <memory>
#include <vector>

namespace {

bool IsSupportedSampleRate(int sample_rate) {
    return sample_rate == 8000 || sample_rate == 16000 ||
            sample_rate == 32000 || sample_rate == 48000;
}

}  // namespace

struct VoiceAecProcessor {
    int sample_rate = 0;
    int frame_size = 0;
    rtc::scoped_refptr<webrtc::AudioProcessing> audio_processing;
    std::vector<int16_t> reference_buffer;
    std::vector<int16_t> microphone_buffer;
};

VoiceAecProcessor* voice_aec_create(int sample_rate,
                                    int noise_suppression_enabled,
                                    int high_pass_filter_enabled) {
    if (!IsSupportedSampleRate(sample_rate)) {
        return nullptr;
    }

    auto processor = std::make_unique<VoiceAecProcessor>();
    processor->sample_rate = sample_rate;
    processor->frame_size = sample_rate / 100;
    processor->audio_processing = webrtc::AudioProcessingBuilder().Create();
    if (!processor->audio_processing) {
        return nullptr;
    }

    webrtc::AudioProcessing::Config config;
    config.echo_canceller.enabled = true;
    config.echo_canceller.mobile_mode = false;
    config.high_pass_filter.enabled = high_pass_filter_enabled != 0;
    config.noise_suppression.enabled = noise_suppression_enabled != 0;
    config.noise_suppression.level =
        webrtc::AudioProcessing::Config::NoiseSuppression::kLow;
    config.gain_controller1.enabled = false;
    config.gain_controller2.enabled = false;
    processor->audio_processing->ApplyConfig(config);

    processor->reference_buffer.resize(processor->frame_size);
    processor->microphone_buffer.resize(processor->frame_size);
    return processor.release();
}

void voice_aec_destroy(VoiceAecProcessor* processor) {
    delete processor;
}

int voice_aec_frame_size(const VoiceAecProcessor* processor) {
    return processor == nullptr ? 0 : processor->frame_size;
}

int voice_aec_process(VoiceAecProcessor* processor,
                        const int16_t* microphone,
                        const int16_t* playback_reference,
                        size_t frame_count,
                        int16_t* output) {
    if (processor == nullptr || microphone == nullptr ||
        playback_reference == nullptr || output == nullptr ||
        frame_count != static_cast<size_t>(processor->frame_size)) {
        return -1;
    }

    std::copy_n(playback_reference, frame_count,
                processor->reference_buffer.begin());
    std::copy_n(microphone, frame_count,
                processor->microphone_buffer.begin());

    const webrtc::StreamConfig stream_config(processor->sample_rate, 1);
    int result = processor->audio_processing->ProcessReverseStream(
        processor->reference_buffer.data(), stream_config, stream_config,
        processor->reference_buffer.data());
    if (result != 0) {
        return result;
    }

    result = processor->audio_processing->ProcessStream(
        processor->microphone_buffer.data(), stream_config, stream_config,
        processor->microphone_buffer.data());
    if (result != 0) {
        return result;
    }

    std::copy_n(processor->microphone_buffer.begin(), frame_count, output);
    return 0;
}
