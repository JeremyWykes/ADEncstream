# Golden packets

Byte-exact encstream packets (protocol version 1), **written by hand from the tables in
the wire-format spec**, not produced by our encoder, so that an error shared by
encoder and decoder is still caught. The C tests (`brick/tests/test_wire.c`) and the Python tests
(`host/tests/test_wire.py`) both decode these files and compare every field with values written
independently in each test, and the C encoder must reproduce them byte for byte.

Format: hex byte pairs separated by white space; `#` starts a comment to the end of the line.

Common values: stream ID `0x11223344` (bytes `44 33 22 11`); header fields little-endian. The DATA
and CONTEXT fixtures describe one layout of 16-byte rows: item 0 `Sys.ServoCount` (u32), item 1 the
gate word (`PowerBrick[0].Chan[0].Status`, Type 0, gate bit 11), item 2 `Motor[1].ActPos` (Type 5
double). Rows: row 0 = ServoCount 0x01020304, gate word 0xA5A508A5 (bit 11 set), position 123.5;
row 1 = 0x01020305, 0xA5A500A5 (bit 11 clear), -1234.5678 (0xC0934A456D5CFAAD).
