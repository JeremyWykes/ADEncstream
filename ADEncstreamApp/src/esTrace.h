/* esTrace.h - receiver traffic traces: record and replay.
 *
 *     file   := "ESTRACE1" record*
 *     record := t_ns u64 | kind u8 | flags u8 | length u16 | payload[length]      (little-endian)
 *
 * Part of ADEncstream's EPICS-free core: portable C++11, no EPICS, no POSIX or Windows calls
 * beyond the C standard library's file functions. encstream's host/encstream_trace.py implements
 * the same format independently.
 */
#ifndef ENCSTREAM_ES_TRACE_H
#define ENCSTREAM_ES_TRACE_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <string>
#include <utility>
#include <vector>

namespace encstream {

struct Endpoint {
    uint8_t addr[4];      // IPv4, network order (a.b.c.d)
    uint16_t port;        // host order
};

bool operator==(const Endpoint &a, const Endpoint &b);

enum TraceKind {
    TRACE_CONFIG = 0,     // "key=value" lines; first record, exactly once
    TRACE_RECEIVED = 1,   // a datagram as received, before any check
    TRACE_SENT = 2,       // a datagram the receiver sent, with its outcome
    TRACE_TIMER = 3,      // an input call without a datagram
    TRACE_END = 4         // end of the observation; last record, exactly once
};

enum {
    TRACE_TRUNCATED = 0x1,    // RECEIVED flags
    TRACE_SEND_FAILED = 0x1   // SENT flags
};

typedef std::vector<std::pair<std::string, std::string> > TraceConfig;

struct TraceRecord {
    uint64_t tNs;
    uint8_t kind;             // TraceKind
    uint8_t flags;
    Endpoint peer;            // RECEIVED: source; SENT: destination
    uint16_t originalLength;  // RECEIVED: what arrived if known (truncated), else the data length
    const uint8_t *data;      // RECEIVED, SENT: the datagram bytes; borrowed from the reader
    size_t dataLength;
    TraceConfig config;       // CONFIG
};

/* Reads a whole trace into memory and hands out its records in order, validating each against
 * the ADEncstream guide: magic, CONFIG first and once, END last and once, non-decreasing times, record
 * extents, known kinds and flags, zero reserved fields. Record data pointers stay valid for the
 * reader's lifetime. */
class TraceReader {
public:
    TraceReader();
    bool openFile(const char *path, std::string *error);
    bool openBuffer(const uint8_t *bytes, size_t length, std::string *error);
    /* the next record: true and *out, or false at the end (then error() is empty if the trace
     * was complete and valid, else the reason) */
    bool next(TraceRecord *out);
    const std::string &error() const { return error_; }
    const TraceConfig &config() const { return config_; }

private:
    bool fail(const std::string &why);
    std::vector<uint8_t> buf_;
    size_t pos_;
    uint64_t last_;
    bool sawConfig_, sawEnd_;
    std::string error_;
    TraceConfig config_;
};

/* Writes a trace. Times are monotonic nanoseconds from the start of the recording; a time below
 * the previous one is stored as the previous one (non-decreasing). Every method returns false on
 * a write error or misuse (record after END, payload over 65535 bytes, CONFIG text with '\n' or
 * a key with '='), after which the trace is unusable. */
class TraceWriter {
public:
    TraceWriter();
    ~TraceWriter();
    bool open(const char *path, const TraceConfig &config, uint64_t tNs, std::string *error);
    bool received(uint64_t tNs, const Endpoint &src, const uint8_t *data, size_t length,
                  bool truncated, uint16_t originalLength);
    bool sent(uint64_t tNs, const Endpoint &dst, const uint8_t *data, size_t length, bool ok);
    bool timer(uint64_t tNs);
    bool end(uint64_t tNs);                   // writes END and closes the file

private:
    bool record(uint64_t tNs, uint8_t kind, uint8_t flags, const uint8_t *head, size_t headLength,
                const uint8_t *data, size_t length);
    TraceWriter(const TraceWriter &);
    TraceWriter &operator=(const TraceWriter &);
    FILE *f_;
    uint64_t last_;
    bool ok_, ended_;
};

}

#endif
