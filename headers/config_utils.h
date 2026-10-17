#ifndef CONFIG_UTILS_H
#define CONFIG_UTILS_H

#include "config.pb.h"
#include <string>

namespace ConfigUtils {
    void load(Config& config);
    bool save(Config& config);
    
    void initUnsetPropertiesWithDefaults(Config& config);

    // headroom: extra capacity reserved so a caller can prepend data (e.g. HTTP headers) without reallocating
    std::string toJSON(const Config& config, size_t headroom = 0);
    bool fromJSON(Config& config, const char* data, size_t dataLen);
    bool fromLegacyStorage(Config& config);
}

#endif
