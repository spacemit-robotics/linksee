/*
 * Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "voice_aec_processor.h"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

namespace {

int16_t NextNoiseSample(uint32_t* state) {
    *state ^= *state << 13;
    *state ^= *state >> 17;
    *state ^= *state << 5;
    return static_cast<int16_t>((*state % 12001) - 6000);
}

double RootMeanSquare(const std::vector<int16_t>& samples) {
    double sum = 0.0;
    for (int16_t sample : samples) {
        const double value = sample;
        sum += value * value;
    }
    return std::sqrt(sum / samples.size());
}

}  // namespace

int main() {
    if (voice_aec_create(44100, 1, 1) != nullptr) {
        std::cerr << "unsupported sample rate was accepted\n";
        return 1;
    }

    VoiceAecProcessor* processor = voice_aec_create(16000, 1, 1);
    if (processor == nullptr || voice_aec_frame_size(processor) != 160) {
        std::cerr << "failed to create 16 kHz AEC processor\n";
        voice_aec_destroy(processor);
        return 1;
    }

    const int frame_size = voice_aec_frame_size(processor);
    std::vector<int16_t> reference(frame_size);
    std::vector<int16_t> microphone(frame_size);
    std::vector<int16_t> output(frame_size);
    uint32_t random_state = 7;
    double input_rms = 0.0;
    double output_rms = 0.0;

    for (int frame_index = 0; frame_index < 400; ++frame_index) {
        for (int index = 0; index < frame_size; ++index) {
            reference[index] = NextNoiseSample(&random_state);
            microphone[index] = static_cast<int16_t>(reference[index] * 0.65);
        }
        if (voice_aec_process(processor, microphone.data(), reference.data(),
                                microphone.size(), output.data()) != 0) {
            std::cerr << "AEC frame processing failed\n";
            voice_aec_destroy(processor);
            return 1;
        }
        if (frame_index >= 300) {
            input_rms += RootMeanSquare(microphone);
            output_rms += RootMeanSquare(output);
        }
    }

    voice_aec_destroy(processor);
    const double attenuation_ratio = output_rms / input_rms;
    if (attenuation_ratio >= 0.30) {
        std::cerr << "insufficient echo attenuation: "
                    << attenuation_ratio << '\n';
        return 1;
    }
    std::cout << "echo attenuation ratio: " << attenuation_ratio << '\n';
    return 0;
}
