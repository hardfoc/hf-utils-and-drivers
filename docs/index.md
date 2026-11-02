---
hide:
  - navigation
  - toc
---

<div class="hf-hero" markdown>
<span class="hf-kicker">HardFOC legacy repository</span>

# Utils and Drivers

<p>This repository is superseded by hf-core. Its handlers moved there, and the driver and utility collections it
pulled in are now hf-core's submodules. New work goes to hf-core.</p>

[hf-core documentation](https://hardfoc.github.io/hf-core/){ .md-button .md-button--primary }
[Where things moved](#where-things-moved){ .md-button }
[hf-core on GitHub](https://github.com/hardfoc/hf-core){ .md-button }
</div>

!!! warning "Not maintained"
    `hf-utils-and-drivers` receives no fixes. Its submodule pins are old, and the handlers here predate the
    BaseCan v2 interface, the host build and the CANopen utilities in hf-core.

## Where things moved

Every file of this repository has a successor in [hardfoc/hf-core](https://github.com/hardfoc/hf-core):

| Here | In hf-core |
| --- | --- |
| `driver-handlers/As5047uHandler.*` | [`handlers/as5047u/`](https://github.com/hardfoc/hf-core/tree/main/handlers/as5047u) |
| `driver-handlers/Bno08xHandler.*` | [`handlers/bno08x/`](https://github.com/hardfoc/hf-core/tree/main/handlers/bno08x) |
| `driver-handlers/NtcTemperatureHandler.*` | [`handlers/ntc/`](https://github.com/hardfoc/hf-core/tree/main/handlers/ntc) |
| `driver-handlers/Pcal95555Handler.*` | [`handlers/pcal95555/`](https://github.com/hardfoc/hf-core/tree/main/handlers/pcal95555) |
| `driver-handlers/Tmc9660Handler.*`, `Tmc9660AdcWrapper.*` | [`handlers/tmc9660/`](https://github.com/hardfoc/hf-core/tree/main/handlers/tmc9660) |
| `driver-handlers/Logger.*` | [`handlers/logger/`](https://github.com/hardfoc/hf-core/tree/main/handlers/logger) |
| `hf-core-drivers` submodule | The same [hf-core-drivers](https://github.com/hardfoc/hf-core-drivers) repository, a submodule of hf-core |
| `hf-core-utils` submodule | The same [hf-core-utils](https://github.com/hardfoc/hf-core-utils) repository, a submodule of hf-core |

hf-core also has handlers this repository never had (ADCs, flow meters, PMIC, secure element, smart LEDs and
more) and the shared code they use in `handlers/common/`.

## Moving a project

<div class="hf-steps" markdown>

1. Replace the submodule:

    ```bash
    git submodule deinit -f external/hf-utils-and-drivers
    git rm external/hf-utils-and-drivers
    git submodule add https://github.com/hardfoc/hf-core.git external/hf-core
    git submodule update --init --recursive
    ```

2. Follow hf-core's integration guide to add it to your CMake or ESP-IDF build, and set its `HF_CORE_ENABLE_*`
   switches for the handlers you use.
3. Change includes from `driver-handlers/<Name>.h` to the handler's folder under `handlers/`.
4. Rebuild and fix what changed in each handler's API; hf-core's handler pages list the current calls.

</div>

The paths in step 1 are an example: use the folder your project keeps submodules in.

Next: [hf-core documentation](https://hardfoc.github.io/hf-core/).
