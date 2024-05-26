#include "utils.h"

std::string IpToString(const std::uint32_t ip_addr)
{
    std::string str{std::to_string(ip_addr & 0xFF)};
    str += ".";
    str += std::to_string((ip_addr >> 8) & 0xFF);
    str += ".";
    str += std::to_string((ip_addr >> 16) & 0xFF);
    str += ".";
    str += std::to_string((ip_addr >> 24) & 0xFF);
    return str;
}