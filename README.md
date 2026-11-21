# Humanoid Modular Control Demo (C++20)

A standalone simulation example extracted from the humanoid robotics document. It models software behavior only and is not hardware-control or safety-certified code.

## Build and run

```sh
cmake -S . -B build
cmake --build build --config Release
./build/humanoid_demo_cpp
./build/section_1_4_locomotion
ctest --test-dir build --output-on-failure
```

## System models

- `humanoid_demo.sysml` contains a SysML v2 textual model of the controller,
  motors, communication interfaces, gait states, and control-loop sequence.
- `humanoid_demo.puml` contains the equivalent UML class and control-flow model.
- `humanoid_demo_uml.svg` is the rendered UML overview for documents and viewers.
- `docs/section-1.4-humanoid-locomotion.md` contains the complete source
  section 1.4, its code fragments, corrected equations, scope, and a
  cross-reference to the executable demo.
- `src/section_1_4_locomotion.cpp` implements the educational section examples
  as a C++20 executable.
