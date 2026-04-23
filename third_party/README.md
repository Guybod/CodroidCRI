# third_party

This project uses local vendored headers for:

- `asio` (standalone, non-Boost)
- `nlohmann/json`

The current environment cannot reach GitHub reliably, so headers are copied
from the local system include paths into:

- `third_party/asio/include`
- `third_party/nlohmann_json/include`

`CMakeLists.txt` includes these folders first to keep dependency resolution local.
