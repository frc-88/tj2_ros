#include "base64_utils.h"
#include <vector>
#include <cstdint>

static const std::string base64_chars =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

std::string base64_encode(const unsigned char* data, size_t input_length) {
    std::string encoded_data;
    encoded_data.reserve(((input_length + 2) / 3) * 4);

    for (size_t i = 0; i < input_length;) {
        uint32_t octet_a = i < input_length ? data[i++] : 0;
        uint32_t octet_b = i < input_length ? data[i++] : 0;
        uint32_t octet_c = i < input_length ? data[i++] : 0;

        uint32_t triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;

        encoded_data.push_back(base64_chars[(triple >> 3 * 6) & 0x3F]);
        encoded_data.push_back(base64_chars[(triple >> 2 * 6) & 0x3F]);
        encoded_data.push_back(base64_chars[(triple >> 1 * 6) & 0x3F]);
        encoded_data.push_back(base64_chars[(triple >> 0 * 6) & 0x3F]);
    }

    size_t mod = input_length % 3;
    if (mod > 0) {
        size_t num_pads = 3 - mod;
        for (size_t i = 0; i < num_pads; i++) {
            encoded_data[encoded_data.size() - 1 - i] = '=';
        }
    }

    return encoded_data;
}

std::string base64_decode(const std::string& input) {
    if (input.length() % 4 != 0) {
        return "";
    }

    size_t padding = 0;
    if (input.length()) {
        if (input[input.length() - 1] == '=') padding++;
        if (input[input.length() - 2] == '=') padding++;
    }

    std::string decoded_data;
    decoded_data.reserve(((input.length() * 3) / 4) - padding);

    std::vector<uint8_t> quadruple(4, 0);
    for (size_t i = 0; i < input.length();) {
        for (size_t j = 0; j < 4; j++) {
            quadruple[j] = (i < input.length()) ? base64_chars.find(input[i++]) : 0;
        }

        uint32_t packed_triplet =
            (quadruple[0] << (3 * 6)) + (quadruple[1] << (2 * 6)) +
            (quadruple[2] << (1 * 6)) + (quadruple[3] << (0 * 6));

        for (size_t j = 0; j < 3; j++) {
            if (i > padding + j) {
                decoded_data.push_back((packed_triplet >> ((2 - j) * 8)) & 0xFF);
            }
        }
    }

    return decoded_data;
}
