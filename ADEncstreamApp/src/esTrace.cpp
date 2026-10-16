/* esTrace.cpp - receiver traffic traces. See esTrace.h. */
#include "esTrace.h"

#include <string.h>

namespace encstream {

namespace {

const char MAGIC[8] = { 'E', 'S', 'T', 'R', 'A', 'C', 'E', '1' };
const size_t REC_HDR = 12;        // t_ns u64, kind u8, flags u8, length u16
const size_t RECV_HDR = 10;       // addr[4], port u16, original_length u16, reserved u16
const size_t SENT_HDR = 8;        // addr[4], port u16, reserved u16

uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

uint64_t get64(const uint8_t *p)
{
    uint64_t v = 0;
    for (int i = 7; i >= 0; --i)
        v = (v << 8) | p[i];
    return v;
}

void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

void put64(uint8_t *p, uint64_t v)
{
    for (int i = 0; i < 8; ++i)
        p[i] = (uint8_t)(v >> (8 * i));
}

}

bool operator==(const Endpoint &a, const Endpoint &b)
{
    return memcmp(a.addr, b.addr, 4) == 0 && a.port == b.port;
}

/* ---- reader ---------------------------------------------------------------------------------- */

TraceReader::TraceReader() : pos_(0), last_(0), sawConfig_(false), sawEnd_(false) {}

bool TraceReader::fail(const std::string &why)
{
    if (error_.empty())
        error_ = why;
    pos_ = buf_.size();
    return false;
}

bool TraceReader::openBuffer(const uint8_t *bytes, size_t length, std::string *error)
{
    buf_.assign(bytes, bytes + length);
    pos_ = 0; last_ = 0; sawConfig_ = sawEnd_ = false; error_.clear(); config_.clear();
    if (length < sizeof MAGIC || memcmp(bytes, MAGIC, sizeof MAGIC) != 0)
        fail("not an ESTRACE1 trace");
    else
        pos_ = sizeof MAGIC;
    if (error)
        *error = error_;
    return error_.empty();
}

bool TraceReader::openFile(const char *path, std::string *error)
{
    std::vector<uint8_t> b;
    FILE *f = fopen(path, "rb");
    if (!f) {
        error_ = std::string("cannot open ") + path;
        if (error)
            *error = error_;
        return false;
    }
    uint8_t chunk[65536];
    size_t n;
    while ((n = fread(chunk, 1, sizeof chunk, f)) > 0)
        b.insert(b.end(), chunk, chunk + n);
    bool readError = ferror(f) != 0;
    fclose(f);
    if (readError) {
        error_ = std::string("error reading ") + path;
        if (error)
            *error = error_;
        return false;
    }
    return openBuffer(b.empty() ? (const uint8_t *)"" : &b[0], b.size(), error);
}

bool TraceReader::next(TraceRecord *out)
{
    if (!error_.empty())
        return false;
    if (pos_ == buf_.size()) {
        if (!sawEnd_)
            return fail(sawConfig_ ? "no END record" : "no CONFIG record");
        return false;
    }
    if (buf_.size() - pos_ < REC_HDR)
        return fail("truncated record header");
    const uint8_t *h = &buf_[pos_];
    uint64_t t = get64(h);
    uint8_t kind = h[8], flags = h[9];
    size_t n = get16(h + 10);
    if (buf_.size() - pos_ - REC_HDR < n)
        return fail("truncated record");
    const uint8_t *p = h + REC_HDR;
    pos_ += REC_HDR + n;
    if (sawEnd_)
        return fail("record after END");
    if (t < last_)
        return fail("time goes backwards");
    last_ = t;

    out->tNs = t;
    out->kind = kind;
    out->flags = flags;
    memset(&out->peer, 0, sizeof out->peer);
    out->originalLength = 0;
    out->data = 0;
    out->dataLength = 0;
    out->config.clear();

    if (kind == TRACE_CONFIG) {
        if (sawConfig_ || flags)
            return fail("CONFIG repeated, or with flags");
        sawConfig_ = true;
        std::string text((const char *)p, n);
        size_t start = 0;
        while (start < text.size()) {
            size_t eol = text.find('\n', start);
            std::string line = text.substr(start, eol == std::string::npos ? std::string::npos
                                                                            : eol - start);
            start = eol == std::string::npos ? text.size() : eol + 1;
            size_t eq = line.find('=');
            if (eq == std::string::npos)
                return fail("CONFIG line without '='");
            out->config.push_back(std::make_pair(line.substr(0, eq), line.substr(eq + 1)));
        }
        config_ = out->config;
        return true;
    }
    if (!sawConfig_)
        return fail("first record is not CONFIG");
    switch (kind) {
    case TRACE_RECEIVED:
        if (n < RECV_HDR || (flags & ~TRACE_TRUNCATED))
            return fail("bad RECEIVED record");
        if (get16(p + 8) != 0)
            return fail("nonzero reserved field");
        memcpy(out->peer.addr, p, 4);
        out->peer.port = get16(p + 4);
        out->originalLength = get16(p + 6);
        out->data = p + RECV_HDR;
        out->dataLength = n - RECV_HDR;
        return true;
    case TRACE_SENT:
        if (n < SENT_HDR || (flags & ~TRACE_SEND_FAILED))
            return fail("bad SENT record");
        if (get16(p + 6) != 0)
            return fail("nonzero reserved field");
        memcpy(out->peer.addr, p, 4);
        out->peer.port = get16(p + 4);
        out->data = p + SENT_HDR;
        out->dataLength = n - SENT_HDR;
        return true;
    case TRACE_TIMER:
    case TRACE_END:
        if (n || flags)
            return fail("TIMER or END with payload or flags");
        if (kind == TRACE_END)
            sawEnd_ = true;
        return true;
    default:
        return fail("unknown record kind");
    }
}

/* ---- writer ---------------------------------------------------------------------------------- */

TraceWriter::TraceWriter() : f_(0), last_(0), ok_(false), ended_(false) {}

TraceWriter::~TraceWriter()
{
    if (f_)
        fclose(f_);
}

bool TraceWriter::open(const char *path, const TraceConfig &config, uint64_t tNs, std::string *error)
{
    std::string text;
    for (size_t i = 0; i < config.size(); ++i) {
        const std::string &k = config[i].first, &v = config[i].second;
        if (k.find('=') != std::string::npos || k.find('\n') != std::string::npos ||
            v.find('\n') != std::string::npos) {
            if (error)
                *error = "CONFIG keys must not contain '=' or newlines, values no newlines";
            return false;
        }
        text += k + "=" + v + "\n";
    }
    if (text.size() > 0xFFFF) {
        if (error)
            *error = "CONFIG too long";
        return false;
    }
    f_ = fopen(path, "wb");
    if (!f_) {
        if (error)
            *error = std::string("cannot create ") + path;
        return false;
    }
    ok_ = fwrite(MAGIC, 1, sizeof MAGIC, f_) == sizeof MAGIC;
    ok_ = ok_ && record(tNs, TRACE_CONFIG, 0, 0, 0, (const uint8_t *)text.data(), text.size());
    if (!ok_ && error)
        *error = std::string("error writing ") + path;
    return ok_;
}

bool TraceWriter::record(uint64_t tNs, uint8_t kind, uint8_t flags, const uint8_t *head,
                         size_t headLength, const uint8_t *data, size_t length)
{
    if (!ok_ || ended_ || !f_ || headLength + length > 0xFFFF)
        return ok_ = false;
    if (tNs < last_)
        tNs = last_;                          // non-decreasing, whatever the caller's clock does
    last_ = tNs;
    uint8_t h[REC_HDR];
    put64(h, tNs);
    h[8] = kind;
    h[9] = flags;
    put16(h + 10, (uint16_t)(headLength + length));
    ok_ = fwrite(h, 1, sizeof h, f_) == sizeof h &&
          (headLength == 0 || fwrite(head, 1, headLength, f_) == headLength) &&
          (length == 0 || fwrite(data, 1, length, f_) == length);
    return ok_;
}

bool TraceWriter::received(uint64_t tNs, const Endpoint &src, const uint8_t *data, size_t length,
                           bool truncated, uint16_t originalLength)
{
    uint8_t h[RECV_HDR];
    memcpy(h, src.addr, 4);
    put16(h + 4, src.port);
    put16(h + 6, truncated ? originalLength
                           : (uint16_t)(length > 0xFFFF ? 0xFFFF : length));
    put16(h + 8, 0);
    return record(tNs, TRACE_RECEIVED, truncated ? TRACE_TRUNCATED : 0, h, sizeof h, data, length);
}

bool TraceWriter::sent(uint64_t tNs, const Endpoint &dst, const uint8_t *data, size_t length, bool ok)
{
    uint8_t h[SENT_HDR];
    memcpy(h, dst.addr, 4);
    put16(h + 4, dst.port);
    put16(h + 6, 0);
    return record(tNs, TRACE_SENT, ok ? 0 : TRACE_SEND_FAILED, h, sizeof h, data, length);
}

bool TraceWriter::timer(uint64_t tNs)
{
    return record(tNs, TRACE_TIMER, 0, 0, 0, 0, 0);
}

bool TraceWriter::end(uint64_t tNs)
{
    bool ok = record(tNs, TRACE_END, 0, 0, 0, 0, 0);
    ended_ = true;
    if (f_) {
        ok = (fclose(f_) == 0) && ok;
        f_ = 0;
    }
    return ok_ = ok;
}

}
