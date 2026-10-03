# bedsim-cpp

Minecraft Bedrock player movement simulation for C++23, ported from
[oomph-ac/bedsim](https://github.com/oomph-ac/bedsim).

## Usage

Your world implementation:

```cpp
class MyWorld {
public:
    bool isLoaded(const bedsim::AABB& area) const;
    bedsim::BlockMovement movement(const bedsim::BlockPos& position) const;
    std::optional<bedsim::Liquid> liquid(const bedsim::BlockPos& position) const;
    void blockCollisions(const bedsim::BlockPos& position, std::vector<bedsim::AABB>& out) const;
};
```

Simulate:

```cpp
bedsim::Simulator simulator{ world, bedsim::predictionOptions() };
bedsim::Body body = currentBody();

const auto result = simulator.simulate(body, { .moveVector = { 0.0f, 1.0f }, .sprintDown = true });
const auto path = simulator.run(body, {}, 200, true);
```

## Build

```
cmake -B build -G Ninja
cmake --build build
ctest --test-dir build
```