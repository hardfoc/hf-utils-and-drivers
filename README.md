# Utils and Drivers (legacy)

**Superseded by [hf-core](https://github.com/hardfoc/hf-core).** This repository is no longer maintained; its
handlers moved to hf-core, and the driver and utility collections it pulled in are hf-core's submodules.

[![Docs](https://github.com/hardfoc/hf-utils-and-drivers/actions/workflows/ci-docs-publish.yml/badge.svg?branch=main)](https://hardfoc.github.io/hf-utils-and-drivers/)
[![License: GPL-3.0](https://img.shields.io/badge/license-GPL--3.0-blue)](LICENSE)

**Use instead: <https://hardfoc.github.io/hf-core/>**

## Where things moved

| Here | In hf-core |
| --- | --- |
| `driver-handlers/As5047uHandler.*` | [`handlers/as5047u/`](https://github.com/hardfoc/hf-core/tree/main/handlers/as5047u) |
| `driver-handlers/Bno08xHandler.*` | [`handlers/bno08x/`](https://github.com/hardfoc/hf-core/tree/main/handlers/bno08x) |
| `driver-handlers/NtcTemperatureHandler.*` | [`handlers/ntc/`](https://github.com/hardfoc/hf-core/tree/main/handlers/ntc) |
| `driver-handlers/Pcal95555Handler.*` | [`handlers/pcal95555/`](https://github.com/hardfoc/hf-core/tree/main/handlers/pcal95555) |
| `driver-handlers/Tmc9660Handler.*`, `Tmc9660AdcWrapper.*` | [`handlers/tmc9660/`](https://github.com/hardfoc/hf-core/tree/main/handlers/tmc9660) |
| `driver-handlers/Logger.*` | [`handlers/logger/`](https://github.com/hardfoc/hf-core/tree/main/handlers/logger) |
| `hf-core-drivers`, `hf-core-utils` submodules | The same repositories, submodules of hf-core |

## Moving a project

```bash
git submodule deinit -f external/hf-utils-and-drivers && git rm external/hf-utils-and-drivers
git submodule add https://github.com/hardfoc/hf-core.git external/hf-core
git submodule update --init --recursive
```

Then follow hf-core's integration guide and change includes from `driver-handlers/<Name>.h` to the handler's
folder under `handlers/`. Details: <https://hardfoc.github.io/hf-utils-and-drivers/>.

## License

GPL-3.0; see [LICENSE](LICENSE).
