# Third-party notices

The project downloads the following pinned build dependencies with CMake
`FetchContent`. They are not relicensed under Apache-2.0.

## MuJoCo 3.16.0

- Source: <https://github.com/google-deepmind/mujoco>
- License: Apache License 2.0
- Project: MuJoCo physics engine
- Modifications: none

## raylib 6.0

- Source: <https://github.com/raysan5/raylib>
- License: zlib/libpng
- Copyright (c) 2013-2026 Ramon Santamaria (@raysan5)
- Modifications: none

## TGUI 1.13.0

- Source: <https://github.com/texus/TGUI>
- License: zlib/libpng
- Copyright (c) 2012-2026 Bruno Van de Velde
- Modifications: one-line raylib 6 glyph-count fix applied at configure time;
 see `cmake/patch_tgui_raylib.cmake`

## JSON for Modern C++ 3.11.3

- Source: <https://github.com/nlohmann/json>
- License: MIT
- Copyright (c) 2013-2026 Niels Lohmann
- Modifications: none

## Noto Sans CJK SC

- File: `assets/fonts/NotoSansCJKsc-Regular.otf`
- Source: <https://github.com/notofonts/noto-cjk>
- License: SIL Open Font License 1.1; see `assets/fonts/OFL.txt`
- Copyright (c) 2010-2026 Adobe and Google
- Modifications: none

The generated binaries may also link to platform libraries selected by raylib.
Those libraries retain their own licenses. Refer to each dependency's source
distribution for its complete license text and bundled third-party notices.
