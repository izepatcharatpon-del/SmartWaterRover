#ifndef USV_FIXED_LOG_BUFFER_H
#define USV_FIXED_LOG_BUFFER_H
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// One owner. Contains complete newline-terminated rows, without overwriting.
template <size_t Capacity> class FixedLogBuffer {
public:
    bool append(const char* row, size_t length, uint32_t now) {
        if (!row || !length || row[length - 1] != '\n' || length > freeBytes()) return false;
        if (!_used) _since = now;
        memcpy(_bytes + _used, row, length);
        _used += length;
        return true;
    }
    size_t size() const { return _used; }
    size_t freeBytes() const { return Capacity - _used; }
    const char* data() const { return _bytes; }
    uint32_t age(uint32_t now) const { return _used ? now - _since : 0; }
    size_t prefix(size_t limit) const {
        size_t count = _used < limit ? _used : limit;
        while (count && _bytes[count - 1] != '\n') --count;
        return count;
    }
    bool consume(size_t count) {
        if (!count || count > _used || _bytes[count - 1] != '\n') return false;
        _used -= count;
        memmove(_bytes, _bytes + count, _used);
        return true;
    }
private:
    char _bytes[Capacity]{};
    size_t _used = 0;
    uint32_t _since = 0;
};
#endif // USV_FIXED_LOG_BUFFER_H
