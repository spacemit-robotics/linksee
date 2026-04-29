/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * NV12 direct drawing utilities (header-only).
 * Draw rectangles and labels directly on NV12 Y/UV planes,
 * avoiding BGR↔NV12 color space conversion overhead.
 */

#ifndef NV12_DRAW_H
#define NV12_DRAW_H

#include <cstdint>
#include <cstring>
#include <algorithm>

namespace nv12_draw {

// YUV color presets
struct YuvColor {
    uint8_t y, u, v;
};

// Common colors in YUV space
constexpr YuvColor kWhite  = {235, 128, 128};
constexpr YuvColor kBlack  = {16,  128, 128};
constexpr YuvColor kRed    = {81,  90,  240};
constexpr YuvColor kGreen  = {145, 54,  34};
constexpr YuvColor kBlue   = {41,  240, 110};
constexpr YuvColor kYellow = {210, 16,  146};
constexpr YuvColor kCyan   = {170, 166, 16};

// Simple 5x7 bitmap font for digits and uppercase letters
// Each character is 5 columns x 7 rows, stored as 7 bytes (each byte = 5 bits, MSB left)
// clang-format off
static const uint8_t kFont5x7[][7] = {
    // '0'
    {0x70, 0x88, 0x98, 0xA8, 0xC8, 0x88, 0x70},
    // '1'
    {0x20, 0x60, 0x20, 0x20, 0x20, 0x20, 0x70},
    // '2'
    {0x70, 0x88, 0x08, 0x10, 0x20, 0x40, 0xF8},
    // '3'
    {0xF8, 0x10, 0x20, 0x10, 0x08, 0x88, 0x70},
    // '4'
    {0x10, 0x30, 0x50, 0x90, 0xF8, 0x10, 0x10},
    // '5'
    {0xF8, 0x80, 0xF0, 0x08, 0x08, 0x88, 0x70},
    // '6'
    {0x30, 0x40, 0x80, 0xF0, 0x88, 0x88, 0x70},
    // '7'
    {0xF8, 0x08, 0x10, 0x20, 0x40, 0x40, 0x40},
    // '8'
    {0x70, 0x88, 0x88, 0x70, 0x88, 0x88, 0x70},
    // '9'
    {0x70, 0x88, 0x88, 0x78, 0x08, 0x10, 0x60},
    // ':'
    {0x00, 0x60, 0x60, 0x00, 0x60, 0x60, 0x00},
    // ' ' (space, index 11)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    // 'A' (index 12)
    {0x70, 0x88, 0x88, 0xF8, 0x88, 0x88, 0x88},
    // 'B'
    {0xF0, 0x88, 0x88, 0xF0, 0x88, 0x88, 0xF0},
    // 'C'
    {0x70, 0x88, 0x80, 0x80, 0x80, 0x88, 0x70},
    // 'D'
    {0xE0, 0x90, 0x88, 0x88, 0x88, 0x90, 0xE0},
    // 'E'
    {0xF8, 0x80, 0x80, 0xF0, 0x80, 0x80, 0xF8},
    // 'F'
    {0xF8, 0x80, 0x80, 0xF0, 0x80, 0x80, 0x80},
    // 'G'
    {0x70, 0x88, 0x80, 0xB8, 0x88, 0x88, 0x70},
    // 'H'
    {0x88, 0x88, 0x88, 0xF8, 0x88, 0x88, 0x88},
    // 'I'
    {0x70, 0x20, 0x20, 0x20, 0x20, 0x20, 0x70},
    // 'J'
    {0x38, 0x10, 0x10, 0x10, 0x10, 0x90, 0x60},
    // 'K'
    {0x88, 0x90, 0xA0, 0xC0, 0xA0, 0x90, 0x88},
    // 'L'
    {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xF8},
    // 'M'
    {0x88, 0xD8, 0xA8, 0x88, 0x88, 0x88, 0x88},
    // 'N'
    {0x88, 0xC8, 0xA8, 0x98, 0x88, 0x88, 0x88},
    // 'O'
    {0x70, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70},
    // 'P'
    {0xF0, 0x88, 0x88, 0xF0, 0x80, 0x80, 0x80},
    // 'Q'
    {0x70, 0x88, 0x88, 0x88, 0xA8, 0x90, 0x68},
    // 'R'
    {0xF0, 0x88, 0x88, 0xF0, 0xA0, 0x90, 0x88},
    // 'S'
    {0x70, 0x88, 0x80, 0x70, 0x08, 0x88, 0x70},
    // 'T'
    {0xF8, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20},
    // 'U'
    {0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70},
    // 'V'
    {0x88, 0x88, 0x88, 0x88, 0x50, 0x50, 0x20},
    // 'W'
    {0x88, 0x88, 0x88, 0x88, 0xA8, 0xD8, 0x88},
    // 'X'
    {0x88, 0x88, 0x50, 0x20, 0x50, 0x88, 0x88},
    // 'Y'
    {0x88, 0x88, 0x50, 0x20, 0x20, 0x20, 0x20},
    // 'Z'
    {0xF8, 0x08, 0x10, 0x20, 0x40, 0x80, 0xF8},
    // '.' (index 49)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60},
    // '-' (index 50)
    {0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x00},
    // '%' (index 51)
    {0xC8, 0xC8, 0x10, 0x20, 0x40, 0x98, 0x98},
};
// clang-format on

inline int char_to_font_index(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c == ':') return 10;
    if (c == ' ') return 11;
    if (c >= 'A' && c <= 'Z') return 12 + (c - 'A');
    if (c >= 'a' && c <= 'z') return 12 + (c - 'a');  // map lowercase to uppercase
    if (c == '.') return 49;
    if (c == '-') return 50;
    if (c == '%') return 51;
    return 11;  // default to space
}

/**
 * Draw a horizontal line on Y plane.
 */
inline void draw_hline_y(uint8_t* y_plane, int stride, int img_h,
    int x1, int x2, int y, uint8_t y_val, int thickness) {
    for (int t = 0; t < thickness; ++t) {
        int row = y + t;
        if (row < 0 || row >= img_h) continue;
        int left = std::max(0, x1);
        int right = std::min(stride - 1, x2);
        if (left > right) continue;
        std::memset(y_plane + row * stride + left, y_val, right - left + 1);
    }
}

/**
 * Draw a vertical line on Y plane.
 */
inline void draw_vline_y(uint8_t* y_plane, int stride, int img_h,
    int x, int y1, int y2, uint8_t y_val, int thickness) {
    for (int t = 0; t < thickness; ++t) {
        int col = x + t;
        if (col < 0 || col >= stride) continue;
        int top = std::max(0, y1);
        int bottom = std::min(img_h - 1, y2);
        for (int row = top; row <= bottom; ++row) {
            y_plane[row * stride + col] = y_val;
        }
    }
}

/**
 * Fill UV values for a horizontal line (NV12: interleaved UV, half resolution).
 */
inline void fill_uv_hline(uint8_t* uv_plane, int uv_stride, int img_h,
    int x1, int x2, int y, uint8_t u_val, uint8_t v_val, int thickness) {
    int uv_h = img_h / 2;
    for (int t = 0; t < thickness; ++t) {
        int uv_row = (y + t) / 2;
        if (uv_row < 0 || uv_row >= uv_h) continue;
        int left = std::max(0, x1 / 2);
        int right = std::min(uv_stride / 2 - 1, x2 / 2);
        uint8_t* row_ptr = uv_plane + uv_row * uv_stride;
        for (int c = left; c <= right; ++c) {
            row_ptr[c * 2]     = u_val;
            row_ptr[c * 2 + 1] = v_val;
        }
    }
}

/**
 * Fill UV values for a vertical line.
 */
inline void fill_uv_vline(uint8_t* uv_plane, int uv_stride, int img_h,
    int x, int y1, int y2, uint8_t u_val, uint8_t v_val, int thickness) {
    int uv_h = img_h / 2;
    for (int t = 0; t < thickness; ++t) {
        int uv_col = (x + t) / 2;
        if (uv_col < 0 || uv_col >= uv_stride / 2) continue;
        int top = std::max(0, y1 / 2);
        int bottom = std::min(uv_h - 1, y2 / 2);
        for (int r = top; r <= bottom; ++r) {
            uv_plane[r * uv_stride + uv_col * 2]     = u_val;
            uv_plane[r * uv_stride + uv_col * 2 + 1] = v_val;
        }
    }
}

/**
 * Draw a rectangle on NV12 frame.
 *
 * @param y_plane   Pointer to Y plane
 * @param uv_plane  Pointer to UV plane (NV12 interleaved)
 * @param stride    Y plane stride (usually == width for NV12)
 * @param img_h     Image height
 * @param x1,y1     Top-left corner
 * @param x2,y2     Bottom-right corner
 * @param color     YUV color
 * @param thickness Line thickness in pixels
 */
inline void draw_rect(uint8_t* y_plane, uint8_t* uv_plane,
    int stride, int img_h,
    int x1, int y1, int x2, int y2,
    YuvColor color, int thickness = 2) {
    // Clamp
    x1 = std::max(0, x1);
    y1 = std::max(0, y1);
    x2 = std::min(stride - 1, x2);
    y2 = std::min(img_h - 1, y2);
    if (x1 >= x2 || y1 >= y2) return;

    int uv_stride = stride;  // NV12: UV stride == Y stride

    // Y plane: 4 edges
    draw_hline_y(y_plane, stride, img_h, x1, x2, y1, color.y, thickness);
    draw_hline_y(y_plane, stride, img_h, x1, x2, y2 - thickness + 1, color.y, thickness);
    draw_vline_y(y_plane, stride, img_h, x1, y1, y2, color.y, thickness);
    draw_vline_y(y_plane, stride, img_h, x2 - thickness + 1, y1, y2, color.y, thickness);

    // UV plane: 4 edges
    fill_uv_hline(uv_plane, uv_stride, img_h, x1, x2, y1, color.u, color.v, thickness);
    fill_uv_hline(uv_plane, uv_stride, img_h, x1, x2, y2 - thickness + 1, color.u, color.v, thickness);
    fill_uv_vline(uv_plane, uv_stride, img_h, x1, y1, y2, color.u, color.v, thickness);
    fill_uv_vline(uv_plane, uv_stride, img_h, x2 - thickness + 1, y1, y2, color.u, color.v, thickness);
}

/**
 * Draw a single character on NV12 Y plane using 5x7 bitmap font.
 * Scale factor controls character size (1 = 5x7, 2 = 10x14, etc.)
 */
inline void draw_char(uint8_t* y_plane, int stride, int img_h,
    int px, int py, char c, uint8_t y_val, int scale = 2) {
    int idx = char_to_font_index(c);
    if (idx < 0 || idx >= static_cast<int>(sizeof(kFont5x7) / sizeof(kFont5x7[0])))
        return;

    const uint8_t* glyph = kFont5x7[idx];
    for (int row = 0; row < 7; ++row) {
        for (int col = 0; col < 5; ++col) {
            if (glyph[row] & (0x80 >> col)) {
                // Fill scaled pixel block
                for (int sy = 0; sy < scale; ++sy) {
                    for (int sx = 0; sx < scale; ++sx) {
                        int x = px + col * scale + sx;
                        int y = py + row * scale + sy;
                        if (x >= 0 && x < stride && y >= 0 && y < img_h) {
                            y_plane[y * stride + x] = y_val;
                        }
                    }
                }
            }
        }
    }
}

/**
 * Draw a text string on NV12 Y plane.
 * Characters are 5*scale wide with 1*scale spacing.
 */
inline void draw_text(uint8_t* y_plane, int stride, int img_h,
    int px, int py, const char* text, uint8_t y_val, int scale = 2) {
    int char_w = 6 * scale;  // 5 pixels + 1 spacing, scaled
    int x = px;
    for (const char* p = text; *p; ++p) {
        draw_char(y_plane, stride, img_h, x, py, *p, y_val, scale);
        x += char_w;
    }
}

/**
 * Draw a labeled rectangle (box + label text above).
 */
inline void draw_labeled_rect(uint8_t* y_plane, uint8_t* uv_plane,
    int stride, int img_h,
    int x1, int y1, int x2, int y2,
    const char* label, YuvColor color, int thickness = 2) {
    draw_rect(y_plane, uv_plane, stride, img_h, x1, y1, x2, y2, color, thickness);

    // Draw label background (filled rect above the box)
    int text_scale = 2;
    int text_h = 7 * text_scale + 4;  // font height + padding
    int label_y = std::max(0, y1 - text_h);

    // Fill background area in Y plane (dark)
    int label_len = static_cast<int>(std::strlen(label));
    int text_w = label_len * 6 * text_scale + 4;
    int label_x2 = std::min(stride - 1, x1 + text_w);

    for (int row = label_y; row < y1 && row < img_h; ++row) {
        int left = std::max(0, x1);
        int right = label_x2;
        if (left < right) {
            std::memset(y_plane + row * stride + left, color.y, right - left);
        }
    }

    // Draw text (white on colored background)
    draw_text(y_plane, stride, img_h, x1 + 2, label_y + 2, label, 235, text_scale);
}

// Color palette for different detection classes
inline YuvColor class_color(int label) {
    static const YuvColor palette[] = {
        kRed, kGreen, kBlue, kYellow, kCyan,
        {180, 80, 200}, {100, 200, 80}, {200, 60, 180},
    };
    int n = sizeof(palette) / sizeof(palette[0]);
    return palette[label % n];
}

}  // namespace nv12_draw

#endif  // NV12_DRAW_H
