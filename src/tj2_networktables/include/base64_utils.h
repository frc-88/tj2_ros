#include <string>

std::string base64_encode(const unsigned char* data, size_t input_length);
std::string base64_decode(const std::string& input);
