/* esReplay - replay a receiver trace through the receiver core and print the shared facts.
 *     esReplay <trace>      key=value lines, as encstream's host/encstream_replay.py prints
 * Exit 0; 1 if a recorded SUBSCRIBE did not match; 2 if the trace could not be replayed. */
#include <stdio.h>

#include "replay.h"

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: esReplay <trace>\n");
        return 2;
    }
    encstream::TraceReader reader;
    std::string error;
    if (!reader.openFile(argv[1], &error)) {
        fprintf(stderr, "esReplay: %s\n", error.c_str());
        return 2;
    }
    replay::Facts facts;
    std::vector<std::string> mismatches;
    if (!replay::run(reader, &facts, &mismatches, &error)) {
        fprintf(stderr, "esReplay: %s\n", error.c_str());
        return 2;
    }
    for (size_t i = 0; i < facts.size(); ++i)
        printf("%s=%s\n", facts[i].first.c_str(), facts[i].second.c_str());
    for (size_t i = 0; i < mismatches.size(); ++i)
        fprintf(stderr, "MISMATCH: %s\n", mismatches[i].c_str());
    return mismatches.empty() ? 0 : 1;
}
