# ADEncstream

An EPICS areaDetector driver that receives an encstream UDP position stream from an Omron Power
Brick LV IMS controller and presents it as NDArrays and process variables.

See the guides in `docs/`:

- `docs/ADENCSTREAM_GUIDE.md` - the module, end to end
- `docs/PVReference.md` - the process-variable reference

Vendored protocol code lives in `vendor/encstream/` (its origin and licence are noted in
`vendor/encstream/VENDOR.md`). Licensed under the EPICS Open License; see `LICENSE`.
